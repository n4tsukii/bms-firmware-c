/* Host-side unit tests. Minimal framework, no dependencies. Build: make test */
#include <stdio.h>
#include <string.h>

#include "bms.h"
#include "bms_can.h"
#include "can.h"

static int g_failed, g_checks;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failed++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); g_checks++; \
    if (_a != _b) { g_failed++; printf("  FAIL %s:%d: %s == %lld, expected %lld\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define RUN(t) do { int before = g_failed; t(); printf("%s %s\n", g_failed == before ? "[ OK ]" : "[FAIL]", #t); } while (0)

/* ---------- helpers ---------- */
static bms_inputs_t nominal(void)
{
    bms_inputs_t in;
    unsigned i;
    memset(&in, 0, sizeof(in));
    for (i = 0u; i < BMS_NUM_CELLS; i++) { in.cell_mv[i] = 3800u; }
    for (i = 0u; i < BMS_NUM_TEMPS; i++) { in.temp_dc[i] = 250; }
    return in;
}

static void steps(bms_t *b, const bms_inputs_t *in, unsigned n)
{
    while (n--) { bms_step(b, in); }
}

static void start_driving(bms_t *b, bms_inputs_t *in)
{
    bms_init(b, in);
    in->ignition = true;
    steps(b, in, 2u);   /* INIT -> STANDBY -> DRIVE */
}

/* ---------- CAN / CRC / queue ---------- */
static void test_crc8_check_value(void)
{
    CHECK_EQ(crc8_j1850((const uint8_t *)"123456789", 9u), 0x4B);
}

static void test_pack_unpack_roundtrip(void)
{
    uint8_t d[8] = {0};
    can_pack_le(d, 4u, 12u, 0xABCu);          /* crosses a byte boundary */
    CHECK_EQ(can_unpack_le(d, 4u, 12u), 0xABC);
    CHECK_EQ(d[0], 0xC0);
    CHECK_EQ(d[1], 0xAB);
    can_pack_le(d, 24u, 16u, (uint32_t)(-423) & 0xFFFFu);
    CHECK_EQ(can_sign_extend(can_unpack_le(d, 24u, 16u), 16u), -423);
    CHECK_EQ(can_unpack_le(d, 4u, 12u), 0xABC);  /* neighbour untouched */
}

/* Reference frame generated with cantools from ev_network.dbc (EV CAN simulator project). */
static void test_status_frame_matches_dbc(void)
{
    static const uint8_t expected[8] = {0x97, 0x48, 0x0F, 0x59, 0xFE, 0x47, 0x72, 0xFB};
    bms_status_msg_t m = {755u, 391200u, -42300, 310, BMS_STATE_DRIVE, 7u};
    bms_status_msg_t back;
    can_frame_t f;
    bms_status_encode(&m, &f);
    CHECK_EQ(f.id, 0x100);
    CHECK_EQ(f.dlc, 8);
    CHECK(memcmp(f.data, expected, 8u) == 0);
    CHECK(bms_status_decode(&f, &back));
    CHECK_EQ(back.soc_dpct, 755);
    CHECK_EQ(back.current_ma, -42300);
    CHECK_EQ(back.max_temp_dc, 310);
    CHECK_EQ(back.state, BMS_STATE_DRIVE);
    f.data[2] ^= 0x10u;                       /* single bit flip */
    CHECK(!bms_status_decode(&f, &back));
}

static void test_queue_full_empty_wrap(void)
{
    can_queue_t q;
    can_frame_t f, out;
    unsigned i, round;
    memset(&f, 0, sizeof(f));
    can_queue_init(&q);
    CHECK(!can_queue_pop(&q, &out));
    for (round = 0u; round < 3u; round++) {    /* indices wrap through the buffer */
        for (i = 0u; i < CAN_TX_QUEUE_SIZE; i++) {
            f.id = round * 100u + i;
            CHECK(can_queue_push(&q, &f));
        }
        CHECK(!can_queue_push(&q, &f));        /* full */
        CHECK_EQ(can_queue_count(&q), CAN_TX_QUEUE_SIZE);
        for (i = 0u; i < CAN_TX_QUEUE_SIZE; i++) {
            CHECK(can_queue_pop(&q, &out));
            CHECK_EQ(out.id, round * 100u + i); /* FIFO order */
        }
        CHECK(!can_queue_pop(&q, &out));
    }
}

/* ---------- SOC ---------- */
static void test_ocv_lookup(void)
{
    CHECK_EQ(bms_soc_from_ocv(2500u), 0);
    CHECK_EQ(bms_soc_from_ocv(4300u), 1000);
    CHECK_EQ(bms_soc_from_ocv(3800u), 550);
    CHECK_EQ(bms_soc_from_ocv(3750u), 450);   /* interpolated between 3700 and 3800 */
}

static void test_coulomb_counting(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    uint16_t soc0;
    start_driving(&b, &in);
    soc0 = b.soc_dpct;
    in.current_ma = 100000;                    /* 100 A for 36 s = 1 Ah = 1 % */
    steps(&b, &in, 3600u);
    CHECK_EQ(soc0 - b.soc_dpct, 10);
    in.current_ma = -100000;                   /* regen gives it back */
    steps(&b, &in, 3600u);
    CHECK_EQ(b.soc_dpct, soc0);
}

/* ---------- state machine ---------- */
static void test_startup_and_drive(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    bms_init(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_INIT);
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_STANDBY);
    CHECK(!b.contactor_closed);
    in.ignition = true;
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_DRIVE);
    CHECK(b.contactor_closed);
    in.ignition = false;
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_STANDBY);
    CHECK(!b.contactor_closed);
}

static void test_charging(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    bms_init(&b, &in);
    bms_step(&b, &in);
    in.charger_connected = true;
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_CHARGING);
    CHECK(b.contactor_closed);
    in.charger_connected = false;
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_STANDBY);
}

static void test_overtemp_debounce(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    start_driving(&b, &in);
    in.temp_dc[3] = 600;                       /* 60.0 degC on one sensor */
    steps(&b, &in, BMS_DEBOUNCE_SET - 1u);
    CHECK_EQ(b.state, BMS_STATE_DRIVE);        /* not yet: glitch filter */
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_FAULT);
    CHECK(b.faults & BMS_FAULT_BIT(BMS_FAULT_IDX_OVERTEMP));
    CHECK(!b.contactor_closed);
}

static void test_single_glitch_ignored(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    unsigned i;
    start_driving(&b, &in);
    for (i = 0u; i < 20u; i++) {               /* spike every other sample */
        in.cell_mv[10] = (i & 1u) ? 4300u : 3800u;
        bms_step(&b, &in);
    }
    CHECK_EQ(b.state, BMS_STATE_DRIVE);
    CHECK_EQ(b.faults, 0);
}

static void test_overtemp_hysteresis(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    start_driving(&b, &in);
    in.temp_dc[0] = 620;
    steps(&b, &in, BMS_DEBOUNCE_SET);
    CHECK(b.faults & BMS_FAULT_BIT(BMS_FAULT_IDX_OVERTEMP));
    in.temp_dc[0] = 580;                       /* below set, above clear */
    steps(&b, &in, 50u);
    CHECK(b.faults & BMS_FAULT_BIT(BMS_FAULT_IDX_OVERTEMP));
    in.temp_dc[0] = 540;                       /* below clear threshold */
    steps(&b, &in, BMS_DEBOUNCE_CLEAR - 1u);
    CHECK(b.faults & BMS_FAULT_BIT(BMS_FAULT_IDX_OVERTEMP));
    bms_step(&b, &in);
    CHECK_EQ(b.faults, 0);
}

static void test_fault_is_latched_until_reset(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    start_driving(&b, &in);
    in.current_ma = 350000;                    /* overcurrent */
    steps(&b, &in, BMS_DEBOUNCE_SET);
    CHECK_EQ(b.state, BMS_STATE_FAULT);
    in.current_ma = 0;
    steps(&b, &in, BMS_DEBOUNCE_CLEAR + 5u);
    CHECK_EQ(b.faults, 0);
    CHECK_EQ(b.state, BMS_STATE_FAULT);        /* still latched */
    CHECK(b.fault_latch & BMS_FAULT_BIT(BMS_FAULT_IDX_OVERCURRENT));
    in.fault_reset = true;                     /* reset refused while ignition on */
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_FAULT);
    in.ignition = false;
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_STANDBY);
    CHECK_EQ(b.fault_latch, 0);
}

static void test_implausible_sensor_at_startup(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    in.cell_mv[17] = 0u;                       /* broken sense wire */
    bms_init(&b, &in);
    in.ignition = true;
    bms_step(&b, &in);
    CHECK_EQ(b.state, BMS_STATE_FAULT);
    CHECK(b.faults & BMS_FAULT_BIT(BMS_FAULT_IDX_SENSOR));
    CHECK(!b.contactor_closed);
}

static void test_undervoltage(void)
{
    bms_t b;
    bms_inputs_t in = nominal();
    start_driving(&b, &in);
    in.cell_mv[95] = 2750u;
    steps(&b, &in, BMS_DEBOUNCE_SET);
    CHECK(b.faults & BMS_FAULT_BIT(BMS_FAULT_IDX_CELL_UV));
    CHECK_EQ(b.min_cell_mv, 2750);
}

int main(void)
{
    RUN(test_crc8_check_value);
    RUN(test_pack_unpack_roundtrip);
    RUN(test_status_frame_matches_dbc);
    RUN(test_queue_full_empty_wrap);
    RUN(test_ocv_lookup);
    RUN(test_coulomb_counting);
    RUN(test_startup_and_drive);
    RUN(test_charging);
    RUN(test_overtemp_debounce);
    RUN(test_single_glitch_ignored);
    RUN(test_overtemp_hysteresis);
    RUN(test_fault_is_latched_until_reset);
    RUN(test_implausible_sensor_at_startup);
    RUN(test_undervoltage);
    printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
