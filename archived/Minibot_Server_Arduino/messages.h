typedef struct {
  uint8_t targetID;
  bool enabled;
  int8_t m0_vel;
  int8_t m1_vel;
} MotTestCommand;

typedef struct {
  uint8_t targetID;
  uint32_t timestamp;
  float x;
  float y;
  float speed;
} PositionCommand;

typedef struct {
  uint8_t targetID;
  uint32_t timestamp;
} PositionRequest;

typedef struct {
  uint8_t responderID;
  uint32_t timestamp;
  float x;
  float y;
} AckMessage;
