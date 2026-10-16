"""
cocotb testbench for ttpe_mem_subsys (ttpe_sram_ctrl + SRAM macro model).

Cycle convention used everywhere in this file
---------------------------------------------
Cycle k starts at rising clock edge k.  Inputs for cycle k are applied right
after edge k and are therefore sampled by the DUT flops at edge k+1.  Outputs
are sampled at the falling edge in the middle of cycle k, so they show flop
state after edge k plus combinational logic driven by cycle-k inputs.
All expectations below are written in these cycle numbers.

Clock is 50 MHz (20 ns).  Every timing check is integer cycle / integer
nanosecond math; no float comparisons are used.  (The single conversion from
simulator time to int ns happens in sample(); the value is an exact multiple.)

Instruction words used here are opaque 32-bit patterns.  The ISA opcodes are
not defined yet, so nothing in this file decodes instructions.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

try:  # cocotb 2.x
    from cocotb.simtime import get_sim_time
except ImportError:  # cocotb 1.x
    from cocotb.utils import get_sim_time

CLK_NS = 20  # 50 MHz

DEFAULTS = dict(
    rst_n=1,
    host_req=0, host_we=0, host_addr=0, host_wdata=0, host_wstrb=0xF,
    host_block=0,
    core_run=0, pc_load=0, pc_load_val=0, instr_accept=0,
)

OUTS = ("host_gnt", "host_wdone", "host_rvalid", "host_rdata",
        "valid_instruction", "instruction", "instr_pc",
        "core_fetch_stall", "core_halted")

# Internal macro-interface probes (read-only)
PROBES = ("A_MEN", "A_REN", "A_WEN", "A_ADDR")

# Opaque firmware words
PROG = {
    0:   0x10010005,
    1:   0x2002000A,
    2:   0x30030000,
    3:   0x40000100,
    4:   0x80040004,
    8:   0x50080008,
    9:   0x60090009,
    100: 0xDEADBEEF,
    511: 0x71FF01FF,
}


# ------------------------------------------------------------------ helpers
def val(sig):
    """Integer value of a signal, or None if it contains X/Z."""
    try:
        return int(sig.value)
    except ValueError:
        return None


def sample(dut):
    s = {k: val(getattr(dut, k)) for k in OUTS}
    for p in PROBES:
        s[p] = val(getattr(dut.u_ctrl, p))
    s["t_ns"] = int(round(get_sim_time("ns")))
    return s


def script(n, *layers):
    """n empty cycles, then apply (cycle_indices, overrides) layers in order."""
    s = [dict() for _ in range(n)]
    for cycles, kw in layers:
        for c in cycles:
            s[c].update(kw)
    return s


async def reset(dut):
    for k, v in DEFAULTS.items():
        getattr(dut, k).value = v
    dut.rst_n.value = 0
    await Timer(1, "ns")  # inputs defined before the first clock edge
    cocotb.start_soon(Clock(dut.clk, CLK_NS, "ns").start())
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def run_cycles(dut, cycles):
    """Apply one dict of input overrides per cycle; return per-cycle samples."""
    trace = []
    for ov in cycles:
        await RisingEdge(dut.clk)
        for k, v in {**DEFAULTS, **ov}.items():
            getattr(dut, k).value = v
        await FallingEdge(dut.clk)
        trace.append(sample(dut))
    # Exact cycle-time check: every cycle must be exactly CLK_NS long.
    for i, t in enumerate(trace):
        elapsed_ns = t["t_ns"] - trace[0]["t_ns"]
        assert elapsed_ns == i * CLK_NS, f"cycle {i}: {elapsed_ns} ns"
    return trace


async def host_write_words(dut, items):
    cyc = [dict(host_req=1, host_we=1, host_addr=a, host_wdata=d, host_wstrb=0xF)
           for a, d in items]
    cyc += [dict() for _ in range(3)]
    return await run_cycles(dut, cyc)


async def load_prog(dut):
    return await host_write_words(dut, sorted(PROG.items()))


def col(tr, name):
    return [t[name] for t in tr]


def flags(n, ones):
    return [1 if i in ones else 0 for i in range(n)]


def check_flag(tr, name, ones):
    exp = flags(len(tr), set(ones))
    got = col(tr, name)
    assert got == exp, f"{name}: got {got}, expected {exp}"


def check_macro(dut):
    """Violation counters exist only in the stand-in model (sim/). The vendor model from the
    PDK has no such counters, so the checks are skipped there. The controller-level probes
    (A_DLY / BIST tie-offs in test_reset_and_idle) still run against both models."""
    if not hasattr(dut.u_sram, "tie_violations"):
        return
    assert int(dut.u_sram.tie_violations.value) == 0, "A_DLY/BIST tie-off violated"
    assert int(dut.u_sram.rw_violations.value) == 0, "REN and WEN together"
    assert int(dut.u_sram.x_violations.value) == 0, "X on macro control/address"


# -------------------------------------------------------------------- tests
@cocotb.test()
async def test_reset_and_idle(dut):
    """Reset state, idle SRAM pins, macro tie-offs."""
    await reset(dut)
    tr = await run_cycles(dut, [dict() for _ in range(8)])
    for name in ("valid_instruction", "host_gnt", "host_rvalid", "host_wdone",
                 "core_fetch_stall", "A_MEN", "A_REN", "A_WEN", "instr_pc"):
        assert col(tr, name) == [0] * 8, name
    assert col(tr, "core_halted") == [1] * 8
    assert val(dut.u_ctrl.A_DLY) == 1
    for n in ("A_BIST_CLK", "A_BIST_EN", "A_BIST_MEN", "A_BIST_WEN",
              "A_BIST_REN", "A_BIST_ADDR", "A_BIST_DIN", "A_BIST_BM"):
        assert val(getattr(dut.u_ctrl, n)) == 0, n
    check_macro(dut)


@cocotb.test()
async def test_host_write_read_latency(dut):
    """Write ack at +1, read data at +2, grant-to-data = exactly 2 cycles."""
    await reset(dut)
    n = 12
    tr = await run_cycles(dut, script(
        n,
        ([0], dict(host_req=1, host_we=1, host_addr=5, host_wdata=0xA5A51234)),
        ([2], dict(host_req=1, host_we=0, host_addr=5)),
    ))
    check_flag(tr, "host_gnt", [0, 2])
    check_flag(tr, "host_wdone", [1])
    check_flag(tr, "host_rvalid", [4])
    assert tr[4]["host_rdata"] == 0xA5A51234
    check_flag(tr, "A_WEN", [0])
    check_flag(tr, "A_REN", [2])
    check_macro(dut)


@cocotb.test()
async def test_host_back_to_back_and_raw(dut):
    """Fully pipelined reads (1/cycle) and write-then-read of same address."""
    await reset(dut)
    await host_write_words(dut, [(10, 0x10101010), (11, 0x11111111),
                                 (12, 0x12121212)])
    n = 16
    tr = await run_cycles(dut, script(
        n,
        ([0], dict(host_req=1, host_addr=10)),
        ([1], dict(host_req=1, host_addr=11)),
        ([2], dict(host_req=1, host_addr=12)),
        ([6], dict(host_req=1, host_we=1, host_addr=20, host_wdata=0xCAFEF00D)),
        ([7], dict(host_req=1, host_we=0, host_addr=20)),
    ))
    check_flag(tr, "host_rvalid", [2, 3, 4, 9])
    assert tr[2]["host_rdata"] == 0x10101010
    assert tr[3]["host_rdata"] == 0x11111111
    assert tr[4]["host_rdata"] == 0x12121212
    assert tr[9]["host_rdata"] == 0xCAFEF00D
    check_flag(tr, "host_wdone", [7])
    check_macro(dut)


@cocotb.test()
async def test_host_byte_strobes(dut):
    """host_wstrb -> A_BM mapping."""
    await reset(dut)
    tr = await run_cycles(dut, script(
        8,
        ([0], dict(host_req=1, host_we=1, host_addr=30,
                   host_wdata=0x11223344, host_wstrb=0xF)),
        ([1], dict(host_req=1, host_we=1, host_addr=30,
                   host_wdata=0xAABBCCDD, host_wstrb=0b0101)),
        ([2], dict(host_req=1, host_we=0, host_addr=30)),
    ))
    check_flag(tr, "host_rvalid", [4])
    assert tr[4]["host_rdata"] == 0x11BB33DD
    check_macro(dut)


@cocotb.test()
async def test_fetch_sequence(dut):
    """core_run at cycle 0 -> first valid at 3; accept -> next valid at +3."""
    await reset(dut)
    await load_prog(dut)
    accepts = [3, 6, 9, 12]
    n = 16
    tr = await run_cycles(dut, script(
        n,
        (range(n), dict(core_run=1)),
        (accepts, dict(instr_accept=1)),
    ))
    check_flag(tr, "valid_instruction", [3, 6, 9, 12, 15])
    for c, a in zip([3, 6, 9, 12, 15], range(5)):
        assert tr[c]["instruction"] == PROG[a], f"cycle {c}"
        assert tr[c]["instr_pc"] == a, f"cycle {c}"
    assert col(tr, "core_halted") == [1] + [0] * (n - 1)
    check_flag(tr, "core_fetch_stall", [])
    check_flag(tr, "A_REN", [1, 4, 7, 10, 13])
    for c, a in zip([1, 4, 7, 10, 13], range(5)):
        assert tr[c]["A_ADDR"] == a
    check_macro(dut)


@cocotb.test()
async def test_fetch_hold_until_accept(dut):
    """valid_instruction and instruction stay stable until accepted."""
    await reset(dut)
    await load_prog(dut)
    n = 18
    tr = await run_cycles(dut, script(
        n,
        (range(n), dict(core_run=1)),
        ([12], dict(instr_accept=1)),
    ))
    check_flag(tr, "valid_instruction", list(range(3, 13)) + [15, 16, 17])
    for c in range(3, 13):
        assert tr[c]["instruction"] == PROG[0] and tr[c]["instr_pc"] == 0
    for c in (15, 16, 17):
        assert tr[c]["instruction"] == PROG[1] and tr[c]["instr_pc"] == 1
    check_macro(dut)


@cocotb.test()
async def test_host_priority_stalls_core(dut):
    """Host wins a contended cycle; core retries and loses exactly 1 cycle."""
    await reset(dut)
    await load_prog(dut)
    n = 8
    tr = await run_cycles(dut, script(
        n,
        (range(n), dict(core_run=1)),
        ([1], dict(host_req=1, host_we=0, host_addr=100)),
    ))
    check_flag(tr, "host_gnt", [1])
    check_flag(tr, "core_fetch_stall", [1])
    assert tr[1]["A_ADDR"] == 100
    assert tr[2]["A_ADDR"] == 0
    check_flag(tr, "A_REN", [1, 2])
    check_flag(tr, "host_rvalid", [3])
    assert tr[3]["host_rdata"] == 0xDEADBEEF
    check_flag(tr, "valid_instruction", [4, 5, 6, 7])  # 1 cycle later than 3
    assert tr[4]["instruction"] == PROG[0]
    check_macro(dut)


@cocotb.test()
async def test_host_block(dut):
    """host_block defers host grants without disturbing the core."""
    await reset(dut)
    await load_prog(dut)
    n = 18
    tr = await run_cycles(dut, script(
        n,
        (range(n), dict(core_run=1)),
        (range(0, 11), dict(host_req=1, host_we=1, host_addr=101,
                            host_wdata=0xBADBAD00)),
        (range(0, 10), dict(host_block=1)),
        ([12], dict(host_req=1, host_we=0, host_addr=101)),
    ))
    check_flag(tr, "host_gnt", [10, 12])
    check_flag(tr, "A_WEN", [10])
    check_flag(tr, "host_wdone", [11])
    check_flag(tr, "core_fetch_stall", [])
    check_flag(tr, "valid_instruction", list(range(3, 18)))
    check_flag(tr, "host_rvalid", [14])
    assert tr[14]["host_rdata"] == 0xBADBAD00
    check_macro(dut)


@cocotb.test()
async def test_jump_flush_prefetched(dut):
    """pc_load while a prefetched instruction is presented."""
    await reset(dut)
    await load_prog(dut)
    n = 15
    tr = await run_cycles(dut, script(
        n,
        (range(n), dict(core_run=1)),
        ([3, 10], dict(instr_accept=1)),
        ([7], dict(pc_load=1, pc_load_val=8)),
    ))
    check_flag(tr, "valid_instruction", [3, 6, 7, 10, 13, 14])
    expect = {3: (PROG[0], 0), 6: (PROG[1], 1), 7: (PROG[1], 1),
              10: (PROG[8], 8), 13: (PROG[9], 9), 14: (PROG[9], 9)}
    for c, (ins, pc) in expect.items():
        assert tr[c]["instruction"] == ins and tr[c]["instr_pc"] == pc, c
    check_macro(dut)


@cocotb.test()
async def test_jump_flush_in_flight(dut):
    """pc_load during WAIT_MEM: stale read data must never be presented."""
    await reset(dut)
    await load_prog(dut)
    n = 9
    tr = await run_cycles(dut, script(
        n,
        (range(n), dict(core_run=1)),
        ([2], dict(pc_load=1, pc_load_val=8)),
    ))
    check_flag(tr, "valid_instruction", [5, 6, 7, 8])
    assert tr[5]["instruction"] == PROG[8] and tr[5]["instr_pc"] == 8
    check_macro(dut)


@cocotb.test()
async def test_jump_flush_on_grant_cycle(dut):
    """pc_load in the same cycle the fetch is granted."""
    await reset(dut)
    await load_prog(dut)
    n = 8
    tr = await run_cycles(dut, script(
        n,
        (range(n), dict(core_run=1)),
        ([1], dict(pc_load=1, pc_load_val=8)),
    ))
    assert tr[1]["A_ADDR"] == 0 and tr[2]["A_ADDR"] == 8
    check_flag(tr, "valid_instruction", [4, 5, 6, 7])
    assert tr[4]["instruction"] == PROG[8] and tr[4]["instr_pc"] == 8
    check_macro(dut)


@cocotb.test()
async def test_pc_load_while_halted(dut):
    """Host sets the start PC while the core is halted."""
    await reset(dut)
    await load_prog(dut)
    n = 9
    tr = await run_cycles(dut, script(
        n,
        (range(1, n), dict(core_run=1)),
        ([0], dict(pc_load=1, pc_load_val=9)),
    ))
    check_flag(tr, "valid_instruction", [4, 5, 6, 7, 8])
    assert tr[4]["instruction"] == PROG[9] and tr[4]["instr_pc"] == 9
    check_macro(dut)


@cocotb.test()
async def test_halt_and_resume_presented(dut):
    """Halt while an unaccepted instruction is presented; resume refetches it."""
    await reset(dut)
    await load_prog(dut)
    n = 13
    tr = await run_cycles(dut, script(
        n,
        ([0, 1, 2, 3, 4], dict(core_run=1)),
        (range(8, n), dict(core_run=1)),
    ))
    check_flag(tr, "valid_instruction", [3, 4, 5, 11, 12])
    check_flag(tr, "core_halted", [0, 6, 7, 8])
    assert tr[11]["instruction"] == PROG[0] and tr[11]["instr_pc"] == 0
    check_macro(dut)


@cocotb.test()
async def test_halt_in_flight(dut):
    """Halt during WAIT_MEM: read data discarded, nothing presented."""
    await reset(dut)
    await load_prog(dut)
    n = 11
    tr = await run_cycles(dut, script(
        n,
        ([0, 1], dict(core_run=1)),
        (range(5, n), dict(core_run=1)),
    ))
    check_flag(tr, "valid_instruction", [8, 9, 10])
    check_flag(tr, "core_halted", [0, 3, 4, 5])
    assert tr[8]["instruction"] == PROG[0] and tr[8]["instr_pc"] == 0
    check_macro(dut)


@cocotb.test()
async def test_accept_and_halt_same_cycle(dut):
    """An instruction accepted in the halting cycle is consumed (PC advances)."""
    await reset(dut)
    await load_prog(dut)
    n = 12
    tr = await run_cycles(dut, script(
        n,
        ([0, 1, 2], dict(core_run=1)),
        (range(6, n), dict(core_run=1)),
        ([3], dict(instr_accept=1)),
    ))
    check_flag(tr, "valid_instruction", [3, 9, 10, 11])
    check_flag(tr, "core_halted", [0, 4, 5, 6])
    assert tr[3]["instruction"] == PROG[0]
    assert tr[9]["instruction"] == PROG[1] and tr[9]["instr_pc"] == 1
    check_macro(dut)


@cocotb.test()
async def test_pc_wraps(dut):
    """PC wraps 511 -> 0."""
    await reset(dut)
    await load_prog(dut)
    n = 8
    tr = await run_cycles(dut, script(
        n,
        ([0], dict(pc_load=1, pc_load_val=511)),
        (range(1, n), dict(core_run=1)),
        ([4], dict(instr_accept=1)),
    ))
    check_flag(tr, "valid_instruction", [4, 7])
    assert tr[4]["instruction"] == PROG[511] and tr[4]["instr_pc"] == 511
    assert tr[7]["instruction"] == PROG[0] and tr[7]["instr_pc"] == 0
    check_macro(dut)


@cocotb.test()
async def test_async_reset_midflight(dut):
    """rst_n clears state without a clock edge and resets the PC to 0."""
    await reset(dut)
    await load_prog(dut)
    n = 12
    tr = await run_cycles(dut, script(
        n,
        (range(1, n), dict(core_run=1)),
        ([0], dict(pc_load=1, pc_load_val=8)),
        ([6], dict(rst_n=0)),
    ))
    # valid at 4,5 (pc 8). At 6 rst_n falls and the flops clear before any
    # clock edge, so the cycle-6 mid-cycle sample already shows idle.
    check_flag(tr, "valid_instruction", [4, 5, 10, 11])
    assert tr[4]["instruction"] == PROG[8] and tr[4]["instr_pc"] == 8
    assert tr[6]["core_halted"] == 1 and tr[6]["instr_pc"] == 0
    assert tr[10]["instruction"] == PROG[0] and tr[10]["instr_pc"] == 0
    check_macro(dut)


@cocotb.test()
async def test_random_traffic(dut):
    """Random host traffic vs. a running core, checked against a scoreboard."""
    await reset(dut)
    rng = random.Random(0xA11CE)

    def pattern(a):
        return ((a * 0x01010101) ^ 0x5A5A0000) & 0xFFFFFFFF

    model = {}
    items = []
    for a in range(512):
        model[a] = pattern(a) if a < 256 else 0
        items.append((a, model[a]))
    await host_write_words(dut, items)

    n = 3000
    cyc = []
    run = 1
    for _ in range(n):
        if rng.random() < 0.01:
            run ^= 1
        d = dict(core_run=run, instr_accept=int(rng.random() < 0.5),
                 host_block=int(rng.random() < 0.25))
        if rng.random() < 0.08:
            d.update(pc_load=1, pc_load_val=rng.randrange(0, 32))
        if rng.random() < 0.5:
            we = int(rng.random() < 0.5)
            addr = rng.randrange(256, 512) if we else rng.randrange(0, 512)
            d.update(host_req=1, host_we=we, host_addr=addr,
                     host_wdata=rng.getrandbits(32),
                     host_wstrb=rng.randrange(0, 16))
        cyc.append(d)

    tr = await run_cycles(dut, cyc)

    rd_exp = {}
    wr_cycles = set()
    n_valid = n_stall = n_reads = n_writes = 0
    for c in range(n):
        v = {**DEFAULTS, **cyc[c]}
        t = tr[c]
        granted = bool(v["host_req"]) and not v["host_block"]
        assert t["host_gnt"] == int(granted), f"cycle {c} host_gnt"
        assert t["host_rvalid"] == (1 if c in rd_exp else 0), f"cycle {c} rvalid"
        if c in rd_exp:
            assert t["host_rdata"] == rd_exp[c], f"cycle {c} rdata"
        assert t["host_wdone"] == (1 if (c - 1) in wr_cycles else 0), f"cycle {c} wdone"
        if granted:
            a = v["host_addr"]
            if v["host_we"]:
                m = 0
                for i in range(4):
                    if (v["host_wstrb"] >> i) & 1:
                        m |= 0xFF << (8 * i)
                model[a] = (model[a] & ~m & 0xFFFFFFFF) | (v["host_wdata"] & m)
                wr_cycles.add(c)
                n_writes += 1
            else:
                rd_exp[c + 2] = model[a]
                n_reads += 1
        if t["valid_instruction"]:
            n_valid += 1
            if t["instr_pc"] < 256:  # static region only
                assert t["instruction"] == model[t["instr_pc"]], f"cycle {c} instr"
        if t["core_fetch_stall"]:
            n_stall += 1
            assert t["host_gnt"] == 1, f"cycle {c} stall without host grant"

    assert n_reads > 100 and n_writes > 100
    assert n_valid > 100
    assert n_stall > 0
    check_macro(dut)
