# PicoCar Project – Continuation / Handoff Notes
(Generated early as a safety net. Paste this whole file into a new chat to continue.)

## 1. What the briefs require (from Project_Write-up.pdf + Project_Briefing.pdf)
- Autonomous car: follow line, decode barcodes (A=Left, B=Right, C=Straight, D=U-turn),
  detect humps + report PEAK hump height (IMU), detect/profile/avoid obstacles
  (ultrasonic on servo: coarse 30/60/90/120/150 deg then fine scan), reacquire line,
  WiFi MQTT telemetry + commands + heartbeat + reconnect.
- MUST: C, micro T-Kernel 3.0 RTOS, Pico W + Cytron Robo Pico, Pico C SDK,
  Barr-C coding standard, follow template repo.
- 5 buddies: B1 WiFi/MQTT, B2 motion (PID, encoders, moveForward/turnLeft/...),
  B3 3xIR line + barcode, B4 IMU hump/tilt/collision/turn-rate/events,
  B5 ultrasonic+servo scan/profile/avoid/recover. Week-10 demo.

## 2. Template repo (confirmed by cloning)
- https://github.com/sirfonzie/mtk3smp-rp2040  (INF2004 lecturer's uT-Kernel 3.0 port)
- GNU make in build_make/. App code = app_program/*.c (flat, wildcard). Entry: usermain().
- Pico SDK **2.2.0** at ../../sdk/pico-sdk relative to build_make (with submodules
  lib/lwip lib/cyw43-driver lib/tinyusb). Newer SDK FAILS (fixed_bitset.h).
- Toolchain arm-none-eabi-gcc 13.2.1 (template baseline); user's Mac has 15.3.1 -> also 0 warnings.
- macOS: build with `gmake`, not /usr/bin/make 3.81 (see 5d).
- Profiles: `make -j8` (UART console GP0/GP1), `CONSOLE=usb_cdc`,
  WiFi: `WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1`
  (needs config/wifi_credentials.h with UTK_WIFI_CREDENTIALS_SET 1).
- Kernel tick CNF_TIMER_PERIOD = 10 ms (config/config.h). SYSCLK 125 MHz.
- Reserved: GP0/GP1 UART console, GP16 liveness LED (BOARD_LED_PIN),
  GP23/24/25/29 radio. SIO spinlocks 0-2 kernel-reserved.
- Kernel drivers DEVCNF_USE_ADC/IIC = 1 in config/config_device.h (disable to own ADC/I2C).
- Free IRQs: TIMER_IRQ_0..3 (0-3), IO_IRQ_BANK0 (13). USE_PTMR=0 so PWM IRQ free.
- IRQ pattern: T_DINT d={.intatr=TA_HLNG,.inthdr=fn}; tk_def_int(n,&d); ClearInt(n); EnableInt(n,2);
- TUs that include <tk/tkernel.h> must NOT include Pico SDK/lwIP headers (size_t clash).
  Pattern: SDK-side .c files expose plain-C (stdint) APIs.
- WiFi service task cyw43_utk_task (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c) owns
  lwIP (NO_SYS). No app hook -> add weak hook call in its poll loop for MQTT.
- MQTT is NOT provided ("left as an exercise").

## 3. Hardware facts
- Robo Pico motors: M1A GP8, M1B GP9, M2A GP10, M2B GP11. A=PWM,B=0 forward; both low=brake.
  PWM max 20 kHz. Servo ports GP12-15. NeoPixel GP18, buttons GP20/21, buzzer GP22.
  Grove: 1=GP0/1, 2=GP2/3 (I2C1, Maker/Qwiic), 3=GP4/5 (I2C0), 4=GP16/17, 5=GP6/26,
  6=GP26/27, 7=GP7/28. ADC: GP26/27/28.
- IMU GY-511 LSM303DLHC: accel 0x19 (CTRL_REG1_A 0x20, CTRL_REG4_A 0x23, OUT 0x28|0x80),
  mag 0x1E (CRA 0x00, CRB 0x01, MR 0x02, OUT_X_H_M 0x03 order X,Z,Y big-endian).
  +-2g HR: 1 mg/LSB (>>4). Mag gain 001: 1100 LSB/G XY, 980 Z. NO GYRO.
- HC-SR04 powered 3.3V per PicoCar slide; uS/58 = cm; >=60 ms cycle.
- IR modules: DO (LM393 comparator) + AO. Slide: pick one.
- Barcode = Code 39, "*X*", ~141 mm long, 20 mm beside an 18 mm line; narrow ~3 mm, wide ~9 mm.
  Sample photo decodes to *A*. A=100001001 B=001001001 C=101001000 D=000011001 *=010010100
  (1=wide, bar/space alternating, 9 elements). Reverse read of *A* = "P1P".

## 4. Design decisions (so far)
- Pin map in app_program/car_config.h (see WIRING in report).
- Hump height = integral of sin(pitch) * encoder distance (no gyro); pitch from accel,
  corrected for longitudinal accel from encoder speed derivative.
- Turn rate from encoders (wR-wL)/track and magnetometer heading derivative.
- Barcode: ADC sampled at 1 kHz by RP2040 TIMER alarm ISR; widths classified per
  character (3 widest of 9 = wide), decode forward then reversed.
- MQTT 3.1.1 minimal client over lwIP raw TCP, SPSC rings between kernel tasks and lwIP task.

## 5. Status (FINAL, turn 2) - everything delivered
- [x] PicoCar_mtk3smp_project.zip: full template repo + app_program (all 5 buddies +
      vehicle integration), tests/host (52/52 pass), docs/REPORT.md + img/, WIRING.md,
      template_changes.patch, firmware/*.uf2 (8 modes, USB console, no WiFi creds).
- [x] Verified: fresh unzip builds with 0 warnings; host tests pass.
- [x] PicoCar_Project_Report.docx (17 pages), PicoCar_Wiring_Diagram.png.
## 5b. Turn 3 additions (items 1 + 2 from the review of the 2024 reference repo)
- Live tuning: app_program/command.c/h (pure parser: start|stop|calibrate|gains|help|
  speed=|pid=kp,ki,kd|ff=kf,offset|line=kp,ki,kd|rate=ms), console_in.c/h (USB ring via
  weak tm_usb_rx_byte hook in lib/libtm/.../usb/usb_tinyusb_glue.c + UART0 FIFO poll via
  hal_uart0_getc; template tm_getchar() is unusable: spins with DI).
  telemetry.c = single executor, 20 ms loop, acks {"type":"cmd"}, gains event 60 ms later.
  motion: motion_set_feedforward, motion_get_gains, motion_set_open_loop, idempotent init.
  line_follow_set/get_gains; vehicle_set_line_gains (vehicle task applies).
  telemetry_init(bool b_stream): test builds get commands too (rate= starts streaming).
- Fixed: TEST_MOTOR created a 2nd motion task per re-run; sweep fought the PID.
- tools/dashboard: picocar_dashboard.py (--mqtt HOST | --serial auto, --headless,
  --snapshot), sim_car.py (MQTT simulator, same formats), README, requirements.
  Logs: logs/<stamp>/telemetry.csv (33 cols), events.csv, heartbeat.csv, commands.log,
  console.log. Verified vs mosquitto+sim and a pty serial emulation.
- Tests 81/81; all builds 0 warnings; patch (5 files) applies to upstream template. (Superseded by 5c.)
- Report updated (4.1 commands + dashboard, 4.2/4.3 tuning procedures, 6, 9, 10, App A/B).

## 5c. Turn 4: BARR-C:2018 conformance pass (user uploaded the standard, said "yes" to full conformance)
- Earlier "follows Barr-C" claim was false: tools/barr_check.py finds 1,570 findings in the turn-3 code.
  Now 0 in app_program/ (54 own files) and tests/host/ (4 files); template-owned
  demo_tasks.c/.h and usb_console_compat.h are skipped (deviation).
- Renames: hal_sdk.c -> hal.c, obstacle_plan.* -> avoidance.* (avoidance_profile/plan/action_name),
  bridge_* -> mqtt_bridge_*, cm_* -> car_math_*, float -> float32_t (car_types.h). New headers:
  car_types.h, hal_irq.h, mqtt_lwip.h, app_main.h; tests/host/test_main.h, hal_stub.h.
- Style: Appendix C layout (`type` / `name (params)` definitions, `/** @file`, `/*!` @brief blocks,
  `/*** end of file ***/`), single exit everywhere, public bodies before private + static prototypes,
  every value cast commented (187), && / || operands parenthesised, volatile on all shared data,
  isfinite() guards, ISRs named *_isr, tasks *_task, handles h_/gh_.
- Tools: app_program/.clang-format (+ copy in tests/host), tools/barr_layout.py, tools/barr_format.sh
  (--check), tools/barr_check.py (token-based; expect "TOTAL (excluding deviations): 0").
- Build flags (subdir.mk): -std=c99 -Wpedantic -Wall -Wextra -Wsign-conversion -Wfloat-equal
  -Wdouble-promotion -Wshadow (no -Wno-unused-parameter); hal.o = gnu11 (SDK headers need C11,
  documented deviation). Host tests use the same flags + -Werror: 87/87 pass.
- Deviations (report 8.2): hal.c C11; usermain/tm_usb_rx_byte/cyw43_utk_app_poll names; kernel types
  in task/ISR signatures; 6.5.a no interrupt keyword (Cortex-M); float32_t use vs 5.4.a;
  (void) discard casts uncommented; end-of-line comments are phrases (2.2.a); template files.
- docs/ABBREVIATIONS.md (1.5.b), docs/templates/module.c/.h (4.4.a, compile clean).
- Defects fixed during the pass (report 8.3): alarm re-arm signed cast; stall-timeout float->uint
  UB; sin/wrap180 range reduction never ending on inf/huge angles; motion status now published
  atomically (private g_work + DI copy, b_last_ok before flag); IR barcode ring barriers.
  Independent review (subagent, old vs new) found no regressions, plus: spurious OBSTACLE at run
  start after imu_reset_run (fixed: only an increase of impacts counts), invalid move now reports
  MOTION_WAIT_FAIL, hal_irq enables a registered pin even if the first tk_def_int failed.
  Printed values now round instead of truncate; app_tests heading printed with 3 decimals.
- Report: intro claim replaced; new section 8 (8.1 checks + before/after table, 8.2 deviations,
  8.3 defects); names/sizes/test counts updated; docx regenerated (pandoc --toc, 25 pages).
- template_changes.patch regenerated: 6 files (adds .gitignore), git apply verified on a fresh clone.
- Verified: fresh unzip builds (8 modes usb_cdc, UART, WiFi+MQTT with temporary creds) 0 warnings.

## 5d. Turn 5 (2026-09-27): review vs briefs -> verified improvement backlog (NOTHING implemented yet)
Verified this turn on the user's Mac (arm-none-eabi-gcc **15.3.1**, not 13.2.1; Pico SDK 2.2.0 was
cloned to a temp scratchpad, NOT into the project; builds done in a scratchpad copy of the repo):
- tests/host 87/87; `tools/barr_check.py` TOTAL 0; all 8 modes (usb_cdc) + UART + WiFi/MQTT build,
  0 warnings (WiFi elf: text 354537, bss 55388).
- macOS `/usr/bin/make` = GNU Make 3.81 -> clean `CONSOLE=usb_cdc` build FAILS ("opening dependency file
  mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/*.d"): 3.81 picks the template's generic
  pico_rp2040/%.o rule (no mkdir) over the usb rule. **Use `gmake`** (Homebrew 4.4.1 works).
- briefs/Barcode Sample.pdf = `*A*` and `*Z*` (Code 39, 3:1; pages 2/4 are half-size copies, ~141 mm).
  Z -> NAV_NONE: reported in telemetry, ignored by vehicle (correct). Spec photo: bars perpendicular
  to line, barcode starts 20 mm beside line edge (single barcode sensor reads ONE side only).

Backlog in priority order (evidence in brackets):
1. HUMP PEAK LOW (mission req 5). terrain.c sum(ds*sin(pitch)) = wheelbase-window MEAN of the profile,
   not its peak (body pitch = chord between drive axle and caster). Sim, real terrain.c, rigid
   2-contact chassis L=130 mm, rear drive, 5.1 mm ticks, LPF 0.3, 1 deg noise: 40x200 mm hump ->
   29.0 mm (-28 %); -8..-43 % for 150..400 mm long humps. Host test models a POINT car so it can't see
   it. Fix (prototyped, within +-3.1 % same conditions): new [CALIBRATE] WHEELBASE_MM; ring buffer of
   front-contact height every 5 mm of odometry, h_f(s) = h_f(s - L) + L*sin(pitch), 5-sample mean,
   peak = max. Then: two-contact host test; report 4.4 table is STALE (says 20.1/30.4/40.1/60.0, test
   prints 20.0/29.9/39.9/60.4) and 6 "within 1.3 %" claim must go.
2. WIFI NOT RECOVERED (B1 "recover connections"). Template cyw43_utk.c:573 calls cyw43_wifi_join()
   once after the boot scan; no retry after a failed join or a link drop (mqtt_lwip.c recovers only
   TCP/MQTT). Fix: in cyw43_utk_task loop, if join_complete and link_status != CYW43_LINK_JOIN for
   > N s, re-join with back-off, count it in heartbeat. Demo: power-cycle the hotspot. (Firmware
   self-reassociation untested.)
3. COLLISION LOOP (plausible; depends on sonar setback). impact -> scan; HC-SR04 blind < US_MIN_MM
   20 -> nothing found -> AVOID_CONTINUE -> car pushes obstacle; velocity mode has no stall timeout.
   Fix: reverse REVERSE_MM before scanning after an impact; velocity stall detector -> OBSTACLE.
4. Front trigger is ONE ping (obstacle.c monitor_task) -> spurious echo = stop + ~2.5 s coarse scan (estimated). Need 2 hits.
5. Servo: no SERVO_INVERT; 0 deg = right assumed. Mirrored mount swaps every left/right decision.
   Add knob + TEST_ULTRASONIC check (box on the right -> closest angle < 90).
6. start_run() never drains the barcode mbuf -> code read while idle becomes first pending command.
7. Hump vs obstacle (hardware test): IMU jerk 700 mg or front sonar may fire at a hump -> stop/scan or
   even bypass it (then no height). Mitigate: ignore triggers while hump state != FLAT; sonar aim.
8. PID: wheel_control adds SPEED_OFFSET_PCT after pid_update(out_max 100) -> windup. Host model, 2 s
   stall then release: +81 % overshoot; out_max = 100 - offset: +55 %. Apply on ff= too.
9. handle_line_search(): STOP logs STOPPED "line not found / end" then "stop command"; its else
   branch is dead. Test gb_abort instead of the state.
10. Add `#if defined(CNF_SMP) && CNF_SMP #error` (DI/EI copies are single-core only; SMP=1 races).
11. Evidence: CPU/loop timing on hardware (report 2.3 is estimates); fill ___ tables; optional GP22
    buzzer beeps on barcode/hump for the demo.
12. Ask/confirm on the real course: barcode side(s); barcode-to-junction distance (NAV_PENDING_MAX_MM
    400 fallback); hump size (20x600 mm raised cosine peaks at 5.98 deg < 6 deg trigger -> never detected).

## 5e. Turn 5b: ponytail over-engineering review (listed only, NOT applied)
Verified by compiling/linking in a scratch copy (template flags, WiFi build):
- lwIP's own MQTT client `$(PICO_SDK_PATH)/lib/lwip/src/apps/mqtt/mqtt.c` compiles with the template's
  lwIP flags/lwipopts (5.2 KB; API mqtt_client_connect(+will, keepalive, user/pass), mqtt_publish,
  mqtt_sub_unsub, mqtt_set_inpub_callback). Could replace mqtt_codec.c/.h (542 lines) + ~340 lines of
  mqtt_lwip.c + test_mqtt. Needs: compile rule in subdir.mk, lwipopts MEMP_NUM_SYS_TIMEOUT +1 and
  MQTT_OUTPUT_RINGBUF_SIZE >= 1024 (telemetry JSON ~400 B).
- [WRONG - see 5f] "newlib libm links if -lm comes after $(OBJS)": true only inside the WiFi image
  (--gc-sections). Plain builds fail: sqrtf/atan2f -> __errno -> impure -> stdio/malloc -> _close,
  _lseek, _read... undefined. Not applied; car_math stays hand-written.
- <ctype.h> isspace/tolower link (+48 B) -> command.c is_space/to_lower (L282-315) can go.
- strtof does NOT link (needs _sbrk/_read/_close... syscalls) -> keep command.c parse_decimal.
Dead/write-only (grep-verified): motor_get_left/right_pct + g_left/right_pct; vehicle_status_t
line_kp/ki/kd; avoidance angle_min/angle_max/forward_mm; line_follow_steer LINE_LOST branch (vehicle
enters LINE_SEARCH first; no test hits it); encoder glitches; bridge b_retain (always false);
terrain_hump_update return value (both callers discard); ir snapshot white/black; imu b_calibrated,
event_seq; barcode last_error; motion_status_t.mode. Estimated total ~ -1,280 lines.

## 5f. Turn 6 (2026-09-27): ponytail review APPLIED (except libm)
Verified: gmake + GCC 15.3.1 + SDK 2.2.0 (scratch clone); 10 builds 0 compiler warnings; host tests
89/89 (-Werror); barr_check TOTAL 0; `tools/barr_format.sh --check` clean (clang-format 18.1.8 via pip).
- MQTT: mqtt_codec.c/.h deleted. mqtt_lwip.c rewritten (243 lines) on lwIP apps/mqtt: static client
  (lwip/apps/mqtt_priv.h); one retry timer: attempt window = RETRY_BASE_MS << g_tries (2, 4, 8 s),
  g_tries reset on CONNACK; on_connection publishes "online" (retained) + subscribes cmd; inpub
  callbacks set before each connect (lwIP keeps them across connects); flush_tx stops on ERR_MEM;
  on_data collects fragments until MQTT_DATA_FLAG_LAST. subdir.mk: OBJS += mtkernel_3/lwip/apps/mqtt/
  mqtt.o (template lwIP pattern rule compiles it). lwipopts.h: MEMP_NUM_SYS_TIMEOUT (8 + TM_WIFI_MQTT),
  MQTT_OUTPUT_RINGBUF_SIZE 1024. Image check: cyw43_utk_task calls our cyw43_utk_app_poll; g_client 1264 B.
  NOT tested against a real broker/hardware yet.
- Host test of the glue: tests/host/lwip/ (fake lwIP; apps/mqtt.h holds all fakes + test controls,
  netif.h/sys.h/apps/mqtt_priv.h forward to it; fake_lwip.c) + tests/host/mqtt_config.h; Makefile adds
  mqtt_lwip.c, lwip/fake_lwip.c, -DTM_WIFI_MQTT=1. test_mqtt (codec) removed, test_mqtt_client added
  (9 checks). Mutation-tested: 5 deliberate bugs all caught (NB: delete picocar_tests before re-running
  in a copied tree - make reuses a copied binary).
- mqtt_bridge: b_retain removed (all callers passed false).
- Dead code removed: motor getters+globals, encoder glitches, vehicle line_kp/ki/kd, avoidance
  angle_min/angle_max/forward_mm, line_follow_steer LOST branch, ir snapshot white/black, imu
  b_calibrated/event_seq, barcode last_error, motion_status_t.mode; terrain_hump_update + hump_track void.
- command.c: is_space/to_lower -> <ctype.h> (links: only the 257-byte _ctype_ table). Its NOTE now gives
  the real reason strtof is not used (needs syscalls); the old size_t claim was wrong for that file.
- libm swap NOT done (my review was wrong, see 5e). car_math.c/.h restored (car_math.h explains why),
  template makefile untouched, template patch still 6 files.
- Numbers: app_program 11056 -> 10079 lines (-977); tests/host +234 (209 = fake lwIP); template +7;
  net -736 (review estimated -1280 incl. libm). Flash: USB/UART builds -1.2 KB; WiFi +3.9 KB (text
  358429; lwIP's client is bigger), RAM +192 B.
- Docs: REPORT.md (1.1, 2.2, 2.3 sizes, libm note, 4.1 MQTT + recovery, 4.4, 4.5, 6 counts + gcc,
  test table, 8 file counts, cast count 167, 8.2 fake-lwIP scope row, App B); PICOCAR_README 89/89;
  arch.dot labels fixed (also stale obstacle_plan/hal_sdk.c) + arch.png re-rendered (graphviz WASM +
  resvg in scratch); PicoCar_Project_Report.docx regenerated: `pandoc docs/REPORT.md -o <docx> --toc
  --resource-path=docs` (pandoc 3.9; reproduces the old docx text exactly from the old md);
  docs/template_changes.patch regenerated vs upstream 2e8e6ad, git apply on a fresh clone gives
  identical files; firmware/*.uf2 rebuilt (8 modes, usb_cdc) and UF2-validated.
- Measurement notes: report's "Application code only" = linked app sections in the WiFi map, KB = 1000:
  25.8 -> 23.8 KB code; RAM 7.7 -> 7.8 KB (old "6.0 KB" not reproducible). Casts (barr_check is_cast):
  orig 189 (report said 187), now 167.
- Still stale in REPORT: 4.4 hump table + 6 "within 1.3 %" (belongs with 5d item 1). 5d backlog untouched.

## 6. What remains (needs the physical car)
- Fill the ___ tables (use dashboard logs): PID step data (TEST_MOTOR), line PID, motion accuracy
  (TEST_MOTION), barcode success rate, hump ruler vs estimate, obstacle profile/bypass.
- Calibrate car_config.h [CALIBRATE] values; set MOTOR_x_INVERT; tune LINE_KP/KD.
- Add wifi_credentials.h + mqtt_config.h and build with WIFI_MQTT=1.
- Fill team names on the report title page.
Known limits: single-channel encoders (no direction sensing), sonar beam widens
obstacles, obstacle length capped 600 mm, tested on SMP=0 profile only.
