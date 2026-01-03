# Motion Controller Implementation Plan

**Project:** Botzo Quadruped Robot - Servo Control System
**Date:** 2025-12-30
**Status:** Servo layer complete, ready for motion controller implementation

---

## Progress Summary

### ✅ Phase 1: Low-Level Servo Layer (COMPLETED)

**Files implemented:**
- `include/servo.h` - Servo API with counter-based calibration
- `src/servo.c` - Full implementation with smooth interpolation
- `include/drivers/pca9685/pca9685.h` - Added `pca9685_get_us_per_count()`
- `src/drivers/pca9685.c` - Precise conversion factor helper
- `src/main.c` - Sine wave test (0-180° smooth motion)

**Key achievements:**
- ✅ Counter-based calibration (maximum precision, single conversion)
- ✅ Quadratic calibration support: `count = a×angle² + b×angle + c`
- ✅ Smooth non-blocking interpolation via scheduler (100Hz updates)
- ✅ Precise timing using actual PCA9685 prescale + oscillator frequency
- ✅ Development helpers (pulse/count conversions)
- ✅ Tested and working with real hardware

**Servo configuration (current):**
```c
servo_config_t config = {
    .channel = 0,
    .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},  // Linear, 270° range
    .min_angle = 0.0f,
    .max_angle = 270.0f,
    .min_count = 72,    // 350μs
    .max_count = 553    // 2700μs
};
```

**Note:** May need calibration adjustment - currently overshoots ~10° at 180°. Consider:
- Option 1: Change to 180° servo (b = 2.672)
- Option 2: Reduce slope (b = 1.686)
- Option 3: Add quadratic term (a = -0.00053, b = 1.877)

---

## 🎯 Phase 2: Motion Controller Layer (NEXT)

### Overview

Implement high-level motion control that:
- Maps robot semantics (legs, joints) to servo channels
- Coordinates multi-servo movements
- Provides interface for future IK/gait planning
- Manages all 12 servos (4 legs × 3 joints)

### Architecture

```
Application (IK/Gait Planner, Protocol Handlers)
           ↓
motion_controller (Robot-aware: legs, joints, coordinated movement)
           ↓
servo (Hardware: individual servo control, interpolation)
           ↓
PCA9685 (PWM driver)
```

---

## Implementation Tasks

### Task 1: Create Data Structures

**File:** `include/motion_controller.h`

```c
// Leg enumeration
typedef enum {
    LEG_FRONT_LEFT = 0,
    LEG_FRONT_RIGHT = 1,
    LEG_BACK_LEFT = 2,
    LEG_BACK_RIGHT = 3,
    NUM_LEGS = 4
} leg_id_e;

// Joint enumeration
typedef enum {
    JOINT_COXA = 0,    // Hip rotation
    JOINT_FEMUR = 1,   // Upper leg
    JOINT_TIBIA = 2,   // Lower leg
    NUM_JOINTS_PER_LEG = 3
} joint_id_e;

// Channel mapping structure
typedef struct {
    uint8_t channel;  // PCA9685 channel (0-15)
} channel_mapping_t;

// Configuration
typedef struct {
    i2c_inst_t *i2c;
    uint8_t pca9685_addr;
    uint16_t pwm_frequency;            // 50Hz
    uint32_t default_interp_time_ms;   // Default transition duration (e.g., 300ms)
} motion_controller_config_t;

// Internal state
typedef struct {
    servo_t servos[12];                                    // All 12 servos
    channel_mapping_t mapping[NUM_LEGS][NUM_JOINTS_PER_LEG];  // Channel map
    motion_controller_config_t config;
    bool initialized;
} motion_controller_state_t;
```

### Task 2: Core API Functions

**File:** `include/motion_controller.h`

```c
// Initialization
ErrorCode_e motion_controller_init(const motion_controller_config_t *config);
ErrorCode_e motion_controller_load_calibrations(const servo_config_t calib[12]);
ErrorCode_e motion_controller_set_mapping(leg_id_e leg, joint_id_e joint, uint8_t channel);

// Control methods
ErrorCode_e motion_controller_move_servo(uint8_t channel, float angle, uint32_t duration_ms);
ErrorCode_e motion_controller_move_joint(leg_id_e leg, joint_id_e joint, float angle, uint32_t duration_ms);
ErrorCode_e motion_controller_move_leg(leg_id_e leg, float coxa, float femur, float tibia, uint32_t duration_ms);
ErrorCode_e motion_controller_move_all(const float angles[12], uint32_t duration_ms);

// State queries
ErrorCode_e motion_controller_get_angle(uint8_t channel, float *angle);
ErrorCode_e motion_controller_get_joint_angle(leg_id_e leg, joint_id_e joint, float *angle);
ErrorCode_e motion_controller_is_moving(bool *is_moving);
ErrorCode_e motion_controller_stop_all(void);

// Scheduler integration
void motion_controller_update_task(void);  // Call at 10ms (100Hz)

// Testing/debug
ErrorCode_e motion_controller_set_count_direct(uint8_t channel, uint16_t count);
ErrorCode_e motion_controller_set_pulse_direct(uint8_t channel, uint16_t pulse_us);
```

### Task 3: Implementation

**File:** `src/motion_controller.c`

**Key implementation notes:**

1. **Singleton state:**
   ```c
   static motion_controller_state_t mc_state = {0};
   ```

2. **Initialization:**
   - Initialize PCA9685 driver
   - Initialize all 12 servo instances
   - Setup default channel mapping (if provided)
   - Validate configuration

3. **Channel mapping:**
   ```c
   ErrorCode_e motion_controller_set_mapping(leg_id_e leg, joint_id_e joint, uint8_t channel) {
       if (leg >= NUM_LEGS || joint >= NUM_JOINTS_PER_LEG || channel >= 12)
           return ERR_INVALID_PARAM;
       mc_state.mapping[leg][joint].channel = channel;
       return OK;
   }

   static uint8_t get_channel(leg_id_e leg, joint_id_e joint) {
       return mc_state.mapping[leg][joint].channel;
   }
   ```

4. **Coordinated movement (key function):**
   ```c
   ErrorCode_e motion_controller_move_leg(leg_id_e leg, float coxa, float femur,
                                          float tibia, uint32_t duration_ms) {
       if (leg >= NUM_LEGS) return ERR_INVALID_PARAM;

       // Use default duration if not specified
       if (duration_ms == 0) {
           duration_ms = mc_state.config.default_interp_time_ms;
       }

       // Get channels for this leg
       uint8_t ch_coxa = get_channel(leg, JOINT_COXA);
       uint8_t ch_femur = get_channel(leg, JOINT_FEMUR);
       uint8_t ch_tibia = get_channel(leg, JOINT_TIBIA);

       // Set all three targets - they interpolate in parallel
       ErrorCode_e err;
       err = servo_set_target(&mc_state.servos[ch_coxa], coxa, duration_ms);
       if (err != OK) return err;

       err = servo_set_target(&mc_state.servos[ch_femur], femur, duration_ms);
       if (err != OK) return err;

       err = servo_set_target(&mc_state.servos[ch_tibia], tibia, duration_ms);
       return err;
   }
   ```

5. **Scheduler task:**
   ```c
   void motion_controller_update_task(void) {
       if (!mc_state.initialized) return;

       // Update all servos (individual mode for now)
       for (uint8_t i = 0; i < 12; i++) {
           servo_update(&mc_state.servos[i]);
       }
   }
   ```

### Task 4: Update CMakeLists.txt

Add `src/motion_controller.c` to SOURCES

### Task 5: Create Test Program

**File:** `src/main.c` (update)

Example test showing coordinated leg movement:

```c
void test_motion_controller(void) {
    static uint8_t step = 0;

    switch(step) {
        case 0:
            printf("Moving front-left leg to home position\n");
            motion_controller_move_leg(LEG_FRONT_LEFT, 0, 90, -45, 500);
            break;
        case 1:
            printf("Moving all legs to standing pose\n");
            float standing[12] = {
                0, 90, -60,  // Front-left
                0, 90, -60,  // Front-right
                0, 90, -60,  // Back-left
                0, 90, -60   // Back-right
            };
            motion_controller_move_all(standing, 1000);
            break;
        case 2:
            printf("Testing single joint (front-left coxa)\n");
            motion_controller_move_joint(LEG_FRONT_LEFT, JOINT_COXA, 45, 500);
            break;
    }

    step = (step + 1) % 3;
}
```

---

## Default Channel Mapping Convention

**Recommended mapping:**
```
Front-Left:  ch0=coxa, ch1=femur, ch2=tibia
Front-Right: ch3=coxa, ch4=femur, ch5=tibia
Back-Left:   ch6=coxa, ch7=femur, ch8=tibia
Back-Right:  ch9=coxa, ch10=femur, ch11=tibia
```

**Setup in main.c:**
```c
motion_controller_set_mapping(LEG_FRONT_LEFT, JOINT_COXA, 0);
motion_controller_set_mapping(LEG_FRONT_LEFT, JOINT_FEMUR, 1);
motion_controller_set_mapping(LEG_FRONT_LEFT, JOINT_TIBIA, 2);
// ... repeat for all legs
```

---

## Calibration Data Management

### Approach 1: Compile-time Array (Initial)

```c
static const servo_config_t default_calibrations[12] = {
    // Front-left leg
    [0] = {  // Coxa
        .channel = 0,
        .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
        .min_angle = -90.0f, .max_angle = 90.0f,
        .min_count = 72, .max_count = 553
    },
    [1] = {  // Femur
        .channel = 1,
        .calib = {.a = 0.0f, .b = 1.781f, .c = 72.0f},
        .min_angle = -45.0f, .max_angle = 135.0f,
        .min_count = 72, .max_count = 553
    },
    // ... define all 12 servos
};

motion_controller_load_calibrations(default_calibrations);
```

### Approach 2: Per-Servo Custom (Future)

When you have real calibration data, update individual servo coefficients.

---

## Performance Optimization (Future)

### Current: Individual Updates
- Each servo = 1 I2C transaction
- 12 servos = 12 transactions/cycle
- Time: ~1.2ms @ 100kHz I2C
- CPU: ~12% per update cycle

### Future: Batch Updates
**When CPU usage becomes critical (>70%)**, implement batch mode:

```c
// In motion_controller.c (future optimization):
void motion_controller_batch_update(void) {
    uint8_t buffer[48];  // 12 channels × 4 bytes

    for (uint8_t i = 0; i < 12; i++) {
        uint16_t count;
        servo_angle_to_count(&mc_state.servos[i],
                            mc_state.servos[i].current_angle,
                            &count);

        buffer[i*4 + 0] = 0;              // ON_L
        buffer[i*4 + 1] = 0;              // ON_H
        buffer[i*4 + 2] = count & 0xFF;   // OFF_L
        buffer[i*4 + 3] = count >> 8;     // OFF_H
    }

    // Single I2C write starting at LED0_ON_L (0x06)
    pca9685_set_pwm_batch(buffer, 12);
}
```

**Benefits:**
- 12 transactions → 1 transaction
- ~1ms saved per cycle
- ~10% CPU reduction

**Required driver addition:**
```c
// In pca9685.h:
ErrorCode_e pca9685_set_pwm_batch(const uint8_t *buffer, uint8_t num_channels);
```

**Design note:** Structure motion_controller to support both modes via flag.

---

## Error Handling

Add to `include/error.h`:
```c
typedef enum ErrorCode_e {
    // ... existing ...
    ERR_SERVO_NOT_INITIALIZED = -9,
    ERR_SERVO_OUT_OF_BOUNDS = -10,
    ERR_SERVO_INVALID_CHANNEL = -11,
    ERR_MOTION_CONTROLLER_NOT_INITIALIZED = -12,
    ERR_INVALID_LEG_ID = -13,
    ERR_INVALID_JOINT_ID = -14,
} ErrorCode_e;
```

---

## Testing Strategy

### Unit Tests (Simulated)

1. **Channel mapping:**
   - Set mapping for all legs/joints
   - Verify correct channel retrieval
   - Test invalid leg/joint IDs

2. **Single servo control:**
   - Move servo by channel
   - Verify angle is set correctly
   - Check state transitions

3. **Joint control:**
   - Move joint by leg/joint ID
   - Verify correct channel is commanded
   - Test invalid IDs

4. **Coordinated movement:**
   - Move entire leg (3 servos)
   - Verify all 3 servos receive targets
   - Check simultaneous interpolation

### Integration Tests (Hardware)

1. **Single leg sweep:**
   - Command leg through full range
   - Verify smooth coordinated motion
   - Check all joints move together

2. **Full-body pose:**
   - Command standing pose (all 12 servos)
   - Verify synchronous arrival at targets
   - Measure timing accuracy

3. **State queries:**
   - Check is_moving() during motion
   - Verify get_angle() returns correct values
   - Test stop_all() emergency stop

### Performance Tests

1. **CPU usage:**
   - Measure with all 12 servos moving
   - Should be <20% at 100Hz updates
   - Monitor for missed scheduler deadlines

2. **Timing accuracy:**
   - Command 500ms movement, measure actual time
   - Verify ±10ms accuracy
   - Check interpolation smoothness

---

## Implementation Checklist

- [ ] Create `include/motion_controller.h` with enums and API
- [ ] Create `src/motion_controller.c` with implementation
- [ ] Add error codes to `include/error.h`
- [ ] Update `CMakeLists.txt` to include motion_controller.c
- [ ] Define default calibration array (12 servos)
- [ ] Implement channel mapping setup
- [ ] Test single servo control
- [ ] Test joint control (leg/joint mapping)
- [ ] Test coordinated leg movement
- [ ] Test full-body pose command
- [ ] Update main.c with motion controller test
- [ ] Register `motion_controller_update_task()` with scheduler
- [ ] Verify smooth coordinated motion on hardware
- [ ] Document channel mapping convention
- [ ] Performance baseline measurement

---

## Next Phase (After Motion Controller)

### Option A: Communication Protocol
- Implement MOVE_SERVO (0x10) command handler
- Implement MOVE_LEG (0x11) command handler
- Enable host control via serial

### Option B: Calibration Tool
- Interactive calibration routine
- Measure actual angles vs commanded
- Calculate quadratic coefficients
- Store/load calibration data

### Option C: IK Integration
- Add inverse kinematics solver
- Foot position → joint angles
- Body pose control
- Integrate with motion controller

---

## Notes & Observations

1. **Current servo calibration** overshoots ~10° at 180° - may need adjustment
2. **CPU usage already at 56%** during sine wave test - batch updates will be important later
3. **Scheduler working well** - 100Hz servo updates, smooth interpolation
4. **Counter-based approach** provides excellent precision
5. **PCA9685 auto-calibration** ensures accurate timing after oscillator drift

---

## References

- Original plan: `/home/pedro/.claude/plans/purrfect-exploring-emerson.md`
- Servo test: `src/main.c` (sine wave test)
- Documentation: `../docs/` (project overview, IK formulas, protocol spec)

---

**Ready to implement when you return!** 🚀
