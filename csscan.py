#!/usr/bin/python

"""
Scan the ROM table and report on CoreSight devices.

Does not do topology (connection) discovery.

To do:
  - CPU affinity and debug/PMU/ETM/CTI grouping
    - replace the ThunderX2-specific hack
  - multiple top-level ROM tables for multi-socket
  - latest CoreSight architecture (mostly done)
  - ETMv3.x/PTF
  - SoC600: TMC, CATU
  - power requestors
"""

from __future__ import print_function

import os, sys, mmap, struct


o_verbose = False
o_show_programming = False 


def bit(x, pos):
    return (x >> pos) & 1


def bits(x, pos, n):
    return (x >> pos) & ((1<<n)-1)


# JEDEC codes: lower 7 bits are the main code, higher bits are the
# continuation code. So Arm is 4 continuation codes followed by 0x3b.
JEDEC_ARM = 0x23b

jedec_designers = {
    JEDEC_ARM:"Arm"
}


# Device architectures defined by Arm (i.e. vaild when Arm is the architect)
# Source: CoreSight Architecture Specification 3.0 Table B2-8
arm_archids = {
    0x1a01:"ITM",
    0x2a04:"v8-M",
    0x6a05:"v8-R",
    0x4a13:"ETMv4",      # REVISION indicates the ETMv4 minor version
    0x1a14:"CTI",
    0x6a15:"v8.0-A",
    0x7a15:"v8.1-A",
    0x8a15:"v8.2-A",
    0x2a16:"PMU",
    0x0a17:"MEM-AP",
    0x0a63:"STM",
    0x0af7:"ROM"
}


def binstr(n,w=None):
    if w is None:
        return "{0:b}".format(n)
    else:
        return ("{0:0%ub}" % w).format(n)


def bits_set(w,m):
    s = []
    for k in sorted(m.keys()):
        if bit(w,k):
            s.append(m[k])
    if s:
        return ' '.join(s)
    else:
        return "-"

assert bits_set(0x011,{0:"x",2:"y",4:"z"}) == "x z"


class Device:
    """
    A single CoreSight device mapped by a ROM table (including ROM tables themselves).    
    """

    def __init__(self, cs, addr):
        self.cs = cs
        assert (addr & 0xfff) == 0, "Device must be located on 4K boundary: 0x%x" % addr
        self.base_address = addr
        # The mmap() base address must be a multiple of the OS page size.
        # But CoreSight devices might be on a smaller granularity.
        # E.g. devices might be at 4K boundaries but the OS is using 64K pages.
        # So we need to adjust the mmap address and size to page granularity.
        # This might mean we end up mapping the same page-sized range several
        # times for different 4K devices located within it.
        self.mmap_offset = addr % cs.page_size        
        mmap_address = addr - self.mmap_offset
        self.m = cs.map(mmap_address)
        self.CIDR = (self.byte(0xFFC)<<24) | (self.byte(0xFF8)<<16) | (self.byte(0xFF4)<<8) | self.byte(0xFF0)
        self.PIDR = (self.byte(0xFD0) << 32) | (self.byte(0xFEC) << 24) | (self.byte(0xFE8) << 16) | (self.byte(0xFE4) << 8) | self.byte(0xFE0)
        self.jedec_designer = (((self.PIDR>>32)&15) << 7) | ((self.PIDR >> 12) & 0x3f)
        # The part number is selected by the component designer.
        self.part_number = self.PIDR & 0xfff
        self.devarch = None
        if self.is_coresight():            
            arch = self.read32(0xFBC)
            if (arch & 0x100000) != 0:
                self.devarch = arch
            self.devtype = self.read32(0xFCC)

    def __del__(self):
        if self.m is not None:
            self.m.close()

    def read32(self, off):
        off += self.mmap_offset
        return struct.unpack("I", self.m[off:off+4])[0]

    def read32x2(self, hi, lo):
        # CoreSight (APB-connected) devices are generally 32-bit wide,
        # and 64-bit values are read as a pair of registers.
        # We assume that we're not dealing with volatile data (e.g. counters)
        # where special action is needed to return a consistent result.
        return (self.read32(hi) << 32) | self.read32(lo)

    def read64(self, off):
        # assume little-endian
        return self.read32x2(off+4,off)

    def read64counter(self, hi, lo):
        # read a live 64-bit counter value from a pair of registers.
        vhia = self.read32(hi)
        while True:
            vlo = self.read32(lo)
            vhib = self.read32(hi)
            if vhia == vhib:
                break
            vhia = vhib
        return (vhib << 32) | vlo

    def byte(self, off):
        off += self.mmap_offset
        return ord(self.m[off])

    def is_arm_part_number(self, n):
        return self.jedec_designer == JEDEC_ARM and self.part_number == n

    def device_class(self):
        return (self.CIDR >> 12) & 15

    def is_coresight(self):
        return self.device_class() == 9

    def is_rom_table(self):
        return self.device_class() == 1 or (self.is_coresight() and self.is_arm_architecture(0x0af7))

    def architect(self):
        # CoreSight devices have a DEVARCH register which specifies the architect and architecture.
        assert self.is_coresight()
        if self.devarch is None:
            return None
        return self.devarch >> 21

    def architecture(self):
        assert self.is_coresight()
        if self.devarch is None:
            return None
        return self.devarch & 0xffff

    def is_arm_architecture(self, arch=None):
        return (self.architect() == JEDEC_ARM) and (arch is None or arch == self.architecture())

    def is_unlocked(self):
        return (self.read32(0xFB4) & 0x02) == 0


class ROMTableEntry:
    """
    An entry in a ROM table. Contains information from the table itself.
    """
    def __init__(self, td, offset, width, e):
        self.table = td            # table device
        self.offset = offset       # byte offset of the entry within the table
        self.width = width         # entry width in bytes
        self.descriptor = e        # the 4-byte or 8-byte table entry (device offset, power req)
        self.device = None         # may be populated later

    def is_present(self):
        return (self.descriptor & 1) != 0
          
    def device_offset(self): 
        # offset is at the top of the word and can be negative
        if self.width == 4:
            off = (self.descriptor & 0xfffff000)
            if (off & 0x80000000) != 0:
                off -= 0x100000000
        else:
            off = (self.descriptor & 0xfffffffffffff000)
            if (off & 0x8000000000000000) != 0:
                off -= 0x10000000000000000
        return off

    def device_address(self):
        return self.table.base_address() + self.device_offset()


class CSROM:
    """
    Container for the overall ROM table scan.
    Owns the mechanism by which we access physical memory - e.g. a
    mapping on to /dev/mem.
    """

    def __init__(self):
        self.page_size = os.sysconf("SC_PAGE_SIZE")
        try:
            self.fd = open("/dev/mem", "r+b")
        except:
            self.fd = open("/dev/csmem", "r+b")      
        self.fno = self.fd.fileno()
        self.device_by_base_address = {}

    def __del__(self):
        self.fd.close()

    def map(self, addr):
        try:
            m = mmap.mmap(self.fno, self.page_size, mmap.MAP_SHARED, mmap.PROT_READ, offset=addr)
        except:
            print("** failed to map 0x%x size 0x%x on fileno %d" % (addr, self.page_size, self.fno))
        return m

    def device(self, addr, rom_table_entry=None):
        assert not addr in self.device_by_base_address, "device at 0x%x already collected" % addr
        d = Device(self, addr)
        d.rom_table_entry = rom_table_entry
        self.device_by_base_address[addr] = d
        return d

    def list_table(self, td, include_empty=False, recurse=True):
        """
        Iterate (perhaps recursively) over a ROM Table, returning
        table entries which contain device objects.

        We assume ROM tables all have the same format. They may have a
        vendor part number, and DEVARCH is not set, but the CIDR device class
        identifies them as a Class 1 ROM table.

        The first entry is at address 0x000. Each subsequent entry is at
        the next 4-byte boundary, until a value of 0x00000000 is read which
        is the final entry.
        """
        if td.is_coresight():
            # Class 9 (new) ROM Table
            etop = 0x800
            devid = td.read32(0xFC8)
            format = devid & 15
            if format == 0:
                ewidth = 4
            elif format == 1:
                ewidth = 8
            else:
                assert False, "unknown Class 9 ROM Table format: %u" % format
        else:
            # Class 1 (old) ROM Table
            etop = 0xF00
            ewidth = 4
        for a in range(0, etop, ewidth):
            if ewidth == 4:
                eword = td.read32(a)
            else:
                eword = td.read64(a)
            if eword == 0:
                break            
            if (eword & 1) == 0 and not include_empty:
                continue
            e = ROMTableEntry(td, a, ewidth, eword)
            if (eword & 1) != 0:
                device_offset = e.device_offset()
                if device_offset == 0:
                    # ROM table points back to itself - shouldn't happen
                    continue
                eaddr = td.base_address + device_offset
                # if we're scanning recursively, we have to map the device even if
                # we aren't otherwise interested in devices. A ROM Table entry doesn't
                # indicate that it points to a sub-table as opposed to some other device -
                # we have to map the device and find out if it's another table.
                d = self.device(eaddr, rom_table_entry=e)
                e.device = d
                yield e
                if recurse and d.is_rom_table():
                    for se in self.list_table(d, include_empty=include_empty, recurse=True):
                        yield se
            else:
                yield e 

    def show_coresight_device(self, d):
        """
        Show some information about the device.
        We organize this to show information progressively from the static
        and abstract, to the dynamic and device-specific:
          - device class: e.g. trace source
          - architecture or product: e.g. ETMv4.1, or CoreSight ETB
          - configuration chosen by designer: e.g. ETMv4.1 with four counters, 16K ETB
          - programming: e.g. ETM counter transition rules, ETF in circular mode
          - state: ETM current counter values, ETB buffer occupancy 
        """

        # architected regs with architected values

        major = d.devtype & 15
        minor = (d.devtype >> 4) & 15

        # Source: CoreSight Architecture Specification 3.0 Table B2-9
        majors = {1:"sink", 2:"link", 3:"source", 4:"control", 5:"logic", 6:"PMU"}
        types = {(1,1):"port", (1,2):"buffer", (1,3):"router",
                 (2,1):"funnel", (2,2):"replicator", (2,3):"fifo",
                 (3,1):"ETM", (3,4):"ITM", (3,6):"STM",
                 (4,1):"CTI", (4,2):"auth", (4,3):"power",
                 (5,1):"core-debug", (5,7):"ELA",
                 (6,1):"PMU (core)", (6,5):"PMU (SMMU)"}
        if (major, minor) in types:
            desc = types[(major, minor)]
        elif major in majors:
            desc = "UNKNOWN %s" % majors[major]
        else:
            desc = "UNKNOWN (devtype = 0x%x)" % (devtype)

        if d.architecture() is None:
            archdesc = "" 
        elif d.is_arm_architecture():
            archid = d.architecture()        
            if archid in arm_archids:
                archdesc = "Arm %s rev%u" % (arm_archids[archid], (d.devarch >> 16)&15)
            else:
                archdesc = "?Arm:0x%04x" % archid
        else:
            archdesc = "?ARCH:0x%x" % d.devarch 

        # architected regs with imp def values
        affinity = d.read32x2(0xFAC,0xFA8)
        devid = d.read32(0xFC8)

        if False:
            authstatus = d.read32(0xFB8)
            for (dix, dom) in enumerate(["NS","S","HN"]):
                for (iix, inv) in enumerate(["I","NI"]):
                    stat = bits(authstatus, dix*4+iix*2, 2)
                    print(" %s%sD=%s" % (dom,inv,["-","?","disabled","enabled"][stat]), end=" ")
            print("auth=%08x" % authstatus, end=" ")
        print("%-14s %-16s" % (desc, archdesc), end=" ")
        if o_verbose:
            print("devid=0x%x" % devid, end=" ")
        if affinity != 0:
            print("aff=0x%x" % affinity, end=" ")
            
        # Now extract additional device-specific information. In general, we can establish
        # the type of device, and our ability to determine further information, in two ways:
        #
        #   - DEVARCH, when present, may indicate that it implements an Arm-defined
        #     architecture (such as ETM, PMU or core debug), irrespective of who it
        #     was designed by.  We can then reference Arm's architecture reference manual
        #     (ARM ARM, or CoreSight architecture manual). Note that DEVARCH might not
        #     be present, indicated by reading as zero.
        #
        #   - PIDR may indicate that it is an Arm-designed device (such as CTI, Funnel etc.
        #     from the CoreSight IP product portfolio), and we can then reference Arm's
        #     product Technical Reference Manual (TRM).
        # 
        # We should always check one or other of DEVARCH and PIDR. It is not sufficient
        # just to look at DEVTYPE.
        # 
        # Some functionality might be implementation-defined (product-specific) even for
        # devices that implement an architecture.

        # Further registers might be in the Core power domain (this will be specified
        # in the manual). Reading powered-off registers might cause a bus error.

        # TBD TBD TBD: avoid probing the PMU of disabled threads on a ThunderX2
        core_powered_off = (d.base_address >= 0x410400000) and (d.base_address & 0x000300000) != 0

        if d.is_arm_architecture() and (d.architecture() & 0x0fff) == 0x0a15:
            # Arm v8 debug architecture: core debug, external view (ED... registers)
            edprsr = d.read32(0x314)
            core_powered_off = ((edprsr & 0x1) == 0)
            if (edprsr & 0x1) == 0:        
                print(" powered-down", end="")
            if (edprsr & 0x4) == 1:
                print(" halted", end="")
            if not core_powered_off:
                dfr = d.read32x2(0xD2C,0xD28)
                if o_verbose:
                    print(" dfr=0x%x bkpt:%u wpt:%u" % (dfr, bits(dfr,12,4)+1, bits(dfr,20,4)+1), end="")
        elif d.is_arm_architecture(0x2a16):
            # Arm architecture: PMU
            # PMU doesn't have a register of its own to indicate power state - you have to
            # find the affine core.
            if not core_powered_off:
                config = d.read32(0xE00)
                n_counters = config & 0xff
                if o_verbose:
                    print(" config:0x%08x" % (config), end="")
                print(" counters:%u" % (n_counters), end="")
                if (config & 0x10000):
                    print(" exported", end="")
                else:
                    print(" not-exported", end="")
        elif d.is_arm_architecture(0x4a13):
            # Arm architecture: ETM
            # Test if the registers are invalid/unreadable, either because the core power domain
            # is powered off or because the ETM hasn't been initialized since reset.            
            pdsr = d.read32(0x314)
            core_powered_off = ((pdsr & 0x1) == 0) or ((pdsr & 0x2) != 0)
            print(" pdsr=0x%08x" % pdsr, end="")
            etmid1 = d.read32(0x1E4)
            if o_verbose:
                print(" etmid1=0x%08x" % etmid1, end="")
            emajor = bits(etmid1,8,4)
            eminor = bits(etmid1,4,4)
            if emajor < 3:             
                earch = "ETMv%u" % (emajor+1)
            elif emajor == 3:
                earch = "PFTv1"
            else:
                earch = "ETMv%u" % (emajor)
            earch += ".%u" % eminor
            print(" %s" % (earch), end="")
            if emajor >= 4:
                etmid0 = d.read32(0x1E0)
                etmid4 = d.read32(0x1F0)
                etmid5 = d.read32(0x1F4)
                if o_verbose:
                    print(" etmid0=0x%08x etmid4=0x%08x etmid5=0x%08x" % (etmid0, etmid4, etmid5), end="")
                if bit(etmid0,5):
                    print(" bb", end="")
                if bit(etmid0,6):
                    print(" cond", end="")
                if bit(etmid0,7):
                    print(" cc", end="")
                if bit(etmid0,9):
                    print(" retstack", end="")
                n_resource_selectors = bits(etmid4,16,4)*2
                n_address_comparator_pairs = bits(etmid4,0,4)
                n_pe_comparators = bits(etmid4,12,4)
                n_events = bits(etmid0,10,2)+1
                n_counters = bits(etmid5,28,3)
                n_seqstates = bits(etmid5,25,3)
                n_extin = bits(etmid5,0,9)
                n_extinsel = bits(etmid5,9,3)
                if eminor >= 3:
                    if bits(etmid4,16,4) == 0:
                        n_events = 0       
                print(" events:%u resources:%u addrcomp:%u pecomp:%u counters:%u seqstates:%u extin:%u extinsel:%u" % (n_events, n_resource_selectors, n_address_comparator_pairs, n_pe_comparators, n_counters, n_seqstates, n_extin, n_extinsel), end="")
                if bit(etmid5,31):
                    print(" reduced-function-counter", end="")
        elif d.is_arm_architecture(0x0a63):
            # CoreSight STM
            n_ports = devid & 0x1ffff
            print(" ports:%u" % n_ports, end="")
        elif d.is_arm_part_number(0x906) or d.is_arm_part_number(0x9ED):
            # CoreSight CTI (SoC400) or CoreSight CTI (SoC600)
            # n.b. SoC600 CTI is fixed at 4 channels
            print(" channels:%u triggers:%u" % (((devid>>16)&0xf), ((devid>>8)&0xff)), end="")
        elif d.is_arm_part_number(0x908):
            # CoreSight trace funnel (SoC400)
            in_ports = devid & 15
            print(" in-ports:%u" % in_ports, end="")
            if (devid & 0xf0) == 3:
                print(" priority-scheme")
        elif d.is_arm_part_number(0x909):
            # CoreSight trace replicator (SoC400)
            out_ports = devid & 15
            print(" out-ports:%u" % out_ports, end="")
            if (devid & 0xf0) == 3:
                print(" priority-scheme")
        elif d.is_arm_part_number(0x912):
            # CoreSight TPIU
            print(" TPIU", end="")
        elif d.is_arm_part_number(0x907):
            # CoreSight ETB
            print(" ETB size:%u" % d.read32(0x000), end="")            
        elif d.is_arm_part_number(0x961):
            # CoreSight TMC (SoC400 generation)
            configtype = (devid >> 6) & 3
            print(" TMC:%s" % ["ETB","ETR","ETF","?3"][configtype], end="")
            if configtype != 1:
                print(" size:%u" % (d.read32(0x004)*4), end="")   # for ETB/ETF this is fixed, for ETR it's the buffer size
            memwidth = (devid >> 8) & 7
            print(" memwidth:%u" % (8<<memwidth), end="")
            if configtype == 1:
                wbdepth = (devid >> 11) & 7
                print(" wb:%u" % (1<<wbdepth), end="")

        # dynamic information, but generic for all CoreSight devices
        if d.is_unlocked():
            print(" unlocked", end="")
        if not core_powered_off:
            claimed = d.read32(0xFA4)
            if claimed:
                print(" claimed:0x%x" % claimed, end="")
        if (d.read32(0xF00) & 1) != 0:
            print(" integration", end="")
        print()

        """
        Show the device programming and current state.

        We might in future separate this into a different routine, but for now
        we rely on some variables we set in the configuration discovery.
        """

        if not o_show_programming:
            return
        if core_powered_off:
            return

        if d.is_arm_architecture(0x4a13):
            # ETM
            if emajor >= 4:
                def res_str(rn):
                    # ETMv4 4.4.2
                    if rn == 0:
                        return "FALSE"
                    elif rn == 1:
                        return "TRUE"
                    else:
                        return "R%u" % rn
                def esel_str(e):
                    if not bit(e,7):
                        return res_str(bits(e,0,5))
                    else:                        
                        pair = bits(e,0,4)*2
                        if bit(e,4) or (bit(e,7) and pair == 0):
                            return "?%x" % e
                        return res_str(pair) + "/" + res_str(pair+1)
                def blist(pfx,bs,n=32,inv=False):
                    bl = []
                    for i in range(0,n):
                        if (not inv and bit(bs,i)) or (inv and not bit(bs,i)):
                            bl.append(pfx+"%u" % i)
                    return bl
                # show stability
                status = d.read32(0x00C)
                if not bit(status,1):
                    print("  unstable")
                if bit(status,0):
                    print("  idle")
                # show main configuration: branch-broadcasting etc.
                enabled = bit(d.read32(0x004),0)
                if enabled:
                    print("  enabled")
                config = d.read32(0x010)
                print("  config: 0x%08x:" % config, end="")
                if bit(config,3):
                    # show which ACs are used to include/exclude branch-broadcast
                    bbctl = d.read32(0x03C)
                    print(" branch-broadcast:0x%x" % (bbctl), end="")  
                if bit(config,4):
                    print(" cycle-count:%u" % (d.read32(0x038)), end="")
                if bit(config,11):
                    tsctl = d.read32(0x030)
                    print(" timestamp-event: %s" % (esel_str(bits(tsctl,0,8))), end="")
                if bit(config,12):
                    print(" return-stack", end="")
                if bit(config,6):
                    print(" cxid", end="")
                if bit(config,7):
                    print(" vmid", end="")
                print()
                # show trace condition (ViewInst control)
                vi = d.read32(0x080)
                event = bits(vi,0,8)
                print("  trace-enable: 0x%08x %s %s" % (vi, esel_str(event), '|'.join(blist("EL",bits(vi,20,3),n=3,inv=True))))
                if n_address_comparator_pairs > 0 or n_pe_comparators > 0:
                    if bit(vi,9):
                        print("  started")
                    else:
                        print("  stopped")
                # show resources
                for rn in range(2,n_resource_selectors):
                    rs = d.read32(0x200+4*rn)
                    if rs == 0:
                        # no external inputs: never true
                        continue
                    print("  resource #%u: 0x%08x" % (rn,rs), end="")
                    if bit(rs,20):
                        print(" INV", end="")
                    if bit(rs,21):
                        print(" PAIRINV", end="")
                    group = bits(rs,16,4)
                    sel = bits(rs,0,16)
                    bl = []
                    if group == 0:
                        bl = blist("EXTSEL",sel)
                    elif group == 1:
                        bl = blist("PECOMP",sel)
                    elif group == 2:
                        bl = blist("SEQ==",sel>>4) + blist("ZERO:C",sel&15)
                    elif group == 3:
                        bl = blist("SSC",sel)
                    elif group == 4:
                        bl = blist("SAC",sel)
                    elif group == 5:
                        bl = blist("AR",sel)
                    elif group == 6:
                        bl = blist("CXID",sel)
                    elif group == 7:
                        bl = blist("VCXID",sel)
                    else:
                        bl = blist("?",sel)
                    print(" %s" % "|".join(bl))
                # Show external inputs.
                if n_extinsel:
                    eis = d.read32(0x120)
                    for ei in range(0,n_extinsel):
                        print("  extin #%u: %u" % (ei, bits(eis,8*ei,8)))
                # Show trace events. These are traced in ETM event packets and as ETM external outputs.
                if n_events:
                    ectl0 = d.read32(0x020)
                    ectl1 = d.read32(0x024)
                    for en in range(0,n_events):
                        if bit(ectl1,en):
                            SEL = bits(ectl0,en*8,8)
                            print("  trace-event #%u: %s" % (en, esel_str(SEL)))
                # show counters                
                for n in range(0,n_counters):
                    reload_value = d.read32(0x140+n*4)
                    control = d.read32(0x150+n*4)
                    value = d.read32(0x160+n*4)
                    if value == 0 and control == 0:
                        # if it's never counting or reloading, probably never programmed
                        continue
                    # print counter details: mostly programming, but the value is current status
                    print("  counter #%u: value:%u reload:%u control:0x%08x" % (n, value, reload_value, control), end="")
                    print(" count:%s reload:%s" % (esel_str(bits(control,0,8)), esel_str(bits(control,8,8))), end="")
                    if bit(control,16):
                        print(" self-reload", end="")
                    if bit(control,17):
                        print(" chain", end="")
                    print()
                if n_seqstates > 1:
                    print("  seqreset: %s" % esel_str(d.read32(0x118)))
                    for sr in range(0,n_seqstates-1):
                        seqev = d.read32(0x100+sr*4)
                        sf = bits(seqev,0,8)
                        sb = bits(seqev,8,8)
                        if sf:
                            print("  seq%u -> seq%u: %s" % (sr, sr+1, esel_str(sf)))
                        if sb:
                            print("  seq%u <- seq%u: %s" % (sr, sr+1, esel_str(sb)))
                    print("  seqstate: %u" % d.read32(0x11C))    # current status
                # probe for integration regs
                if True:
                    any_integration_reg = False
                    for a in range(0xE80,0xFA0,4):                        
                        r = d.read32(a)
                        if r != 0:
                            print("  @%03x: 0x%08x" % (a,r))
                            any_integration_reg = True
                    if not any_integration_reg:
                        print("  no integration registers set")
            else:
                # TBD show older ETMs
                pass
        elif d.is_arm_part_number(0x907) or d.is_arm_part_number(0x961):
            # CoreSight SoC400 ETB or TMC
            is_TMC = d.is_arm_part_number(0x961)
            if not is_TMC:
                configtype = (devid >> 6) & 3
                is_ETR = (configtype == 1)
            else:
                is_ETR = False
            mode = d.read32(0x028) & 3
            print("  mode: %s" % ["circular buffer","software FIFO","hardware FIFO","?3"][mode])
            if is_ETR:
                axi_control = d.read32(0x110)
                print("  AXI control: 0x%08x" % axi_control)
                scatter_gather = bit(axi_control,7)
                print("  buffer address: 0x%x" % d.read64(0x118))                
                if not scatter_gather:
                    print("  buffer size: 0x%x" % (d.read32(0x004)*4))
            ffcr = d.read32(0x304)
            ffcr_map = {0:"formatting",1:"format-triggers",5:"flush-on-trigger",12:"stop-on-flush",13:"stop-on-trigger"}
            print("  flush control: %s" % bits_set(ffcr,ffcr_map))
            # from here, report current status
            if bit(d.read32(0x020),0):
                print("  enabled")
                print("  buffer fill level (current): 0x%08x" % d.read32(0x030))
            else:
                print("  disabled")
            status = d.read32(0x00C)
            if not is_TMC:
                print("  status: %s" % bits_set(status,{0:"full",1:"triggered",2:"AcqComp",3:"FtEmpty"}))
            else:
                print("  status: %s" % bits_set(status,{0:"full",1:"triggered",2:"TMCready",3:"FtEmpty",4:"empty",5:"MemErr"}))
            if False:
                # Reading the latched fill level returns the max fill level since
                # it was last read, and also updates with the current fill level.
                print("  buffer fill level (latched): 0x%08x" % d.read32(0x02C))
        elif d.is_arm_part_number(0x906):
            # CoreSight CTI
            n_trigs = (devid>>8) & 0xff
            n_channels = (devid>>16) & 0xf
            print("  trigger inputs:  %s" % (binstr(d.read32(0x130),n_trigs)))
            print("  trigger outputs: %s" % (binstr(d.read32(0x134),n_trigs)))
            print("  channel inputs:  %s" % (binstr(d.read32(0x138),n_channels)))
            print("  channel outputs: %s" % (binstr(d.read32(0x13C),n_channels)))
        elif d.is_arm_part_number(0x908):
            # CoreSight funnel
            ctrl = d.read32(0x000)
            print("  ports enabled: %s" % (binstr((ctrl & 0xff),in_ports)))
            print("  hold time: %u" % bits(ctrl,8,4))
        elif d.is_arm_part_number(0x909):
            # CoreSight replicator
            print("  id filter port 0: 0x%x" % d.read32(0x000))
            print("  id filter port 1: 0x%x" % d.read32(0x004))
        else:
            # unknown device
            pass


    def show_device(self, d):
        """
        Show device details on a single line.
        """
        rev = (d.PIDR >> 20) & 15       # PIDR2.REVISION
        patch = (d.PIDR >> 28) & 15     # PIDR3.REVAND
        print("@0x%x " % (d.base_address), end=" ")
        print("  0x%03x 0x%03x r%u.%u  " % (d.jedec_designer, d.part_number, rev, patch), end="")
        if (d.CIDR & 0xffff0fff) != 0xb105000d:
            print("unexpected CIDR: 0x%08x" % d.CIDR)
        if d.is_rom_table():            
            print("ROM table")
        elif d.is_coresight():
            self.show_coresight_device(d)
        elif d.device_class() == 0xF and d.is_arm_part_number(0x101):
            print("CoreSight timestamp generator")
            if o_show_programming:
                ctrl = d.read32(0x000)
                print("  %s" % ["disabled","enabled"][bit(ctrl,0)])
                print("  frequency: %uHz" % d.read32(0x020))
                count = d.read64counter(0x00C,0x008)
                print("  time: %u" % count)
        elif d.device_class() == 0xF:
            # might be worth reading DEVARCH even though Class 0xF doesn't guarantee to have it (and it might not be readable)
            print("generic PrimeCell peripheral: DEVARCH=0x%08x DEVAFF=0x%08x" % (d.read32(0xFBC), d.read32(0xFA8)))
            # might be Arm RAS architecture
            if False and d.read32(0xFBC) == 0x47700a00:
                print("  0xE80: %08x" % d.read32(0xE80))
                print("  0xFC8: %08x" % d.read32(0xFC8))
                for a in range(0,0x030,8):
                    print("  0x%03x: %08x" % (a, d.read32(a)))
        else: 
            print("class:%u" % (d.device_class()))


def scan_rom(c, table_addr):
    """
    Scan a ROM Table recursively, showing devices as we go.
    """
    table = c.device(table_addr)
    c.show_device(table)
    for e in c.list_table(table):
        assert e.device is not None
        # TBD break out after first few devices
        if e.device.base_address >= 0x410800000:
            break
        c.show_device(e.device)


if __name__ == "__main__":
    c = CSROM()
    for arg in sys.argv[1:]:
        if arg == "-v":
            o_verbose = True
        elif arg == "--status":
            o_show_programming = True
        else:
            table_addr = int(arg, 16)
            scan_rom(c, table_addr)

