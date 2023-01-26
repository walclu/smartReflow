#define SAMPLE_TIME 0.5f

/*
Lead-Free Reflow Curve
======================
Temperature (Degree Celcius)
245-|                                               x  x
    |                                            x   |     x
    |                                         x      |        x
    |                                      x         |           x
130-|                                   x            |              x
    |                              x    |            |              |   x
    |                         x         |            |              |       x
    |                    x              |            |              |
 90-|               x                   |            |              |
    |             x |                   |            |              |
    |           x   |                   |            |              |
    |         x     |                   |            |              |
    |       x       |                   |            |              |
    |     x         |                   |            |              |
    |   x           |                   |            |              |
30 -| x             |                   |            |              |
    |  <  90 s  >|<          90 s      >| <   30   > |
    | Preheat Stage |   Soaking Stage   | Ramp Stage |              | Cool
 0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ |_ _ _ _ _ _ _ |_ _ _ _ _
                                                               Time (Seconds)
 */

#define CONCAT(A, B) A ## B

#define PPCAT(A, B) CONCAT(A, B)



// TS391LT50

const float TS391LT50_PREHEAT_START_TIME  =		0;
const float TS391LT50_PREHEAT_DURATION	=		90.f;
const float TS391LT50_PREHEAT_START_TEMP 	=		30.f;
const float TS391LT50_PREHEAT_END_TEMP 	=		90.f;

const float TS391LT50_SOAK_START_TIME 	=		TS391LT50_PREHEAT_START_TIME+TS391LT50_PREHEAT_DURATION+SAMPLE_TIME;
const float TS391LT50_SOAK_DURATION 		=		90.f;
const float TS391LT50_SOAK_START_TEMP 	=		90.f;
const float TS391LT50_SOAK_END_TEMP 		=		130.f;

const float TS391LT50_RAMP_START_TIME 	=		TS391LT50_SOAK_START_TIME+TS391LT50_SOAK_DURATION+SAMPLE_TIME;
const float TS391LT50_RAMP_DURATION 		=		30.f;
const float TS391LT50_RAMP_START_TEMP 	=		130.f;
const float TS391LT50_RAMP_END_TEMP 		=		138.f;

const float TS391LT50_REFLOW_START_TIME 	=		TS391LT50_RAMP_START_TIME+TS391LT50_RAMP_DURATION+SAMPLE_TIME;
const float TS391LT50_REFLOW_DURATION		=		30.f;
const float TS391LT50_REFLOW_START_TEMP 	=		138.f;
const float TS391LT50_REFLOW_END_TEMP 	=		165.f;

const float TS391LT50_COOL_START_TIME 	=		TS391LT50_REFLOW_START_TIME + TS391LT50_REFLOW_DURATION+SAMPLE_TIME;
const float TS391LT50_COOL_DURATION		=		30.f;
const float TS391LT50_COOL_END_TEMP 		=		138.f;

const float TS391LT50_PREHEAT_TEMP_INCREASE 	=	((TS391LT50_PREHEAT_END_TEMP-TS391LT50_PREHEAT_START_TEMP)/TS391LT50_PREHEAT_DURATION);
const float TS391LT50_SOAK_TEMP_INCREASE 		= 	((TS391LT50_SOAK_END_TEMP-TS391LT50_SOAK_START_TEMP)/TS391LT50_SOAK_DURATION);
const float TS391LT50_RAMP_TEMP_INCREASE 		=	((TS391LT50_RAMP_END_TEMP-TS391LT50_RAMP_START_TEMP)/TS391LT50_RAMP_DURATION);
const float TS391LT50_REFLOW_TEMP_INCREASE 	=	((TS391LT50_REFLOW_END_TEMP-TS391LT50_REFLOW_START_TEMP)/TS391LT50_RAMP_DURATION);
const float TS391LT50_COOL_TEMP_DECREASE 		=	((TS391LT50_REFLOW_END_TEMP-TS391LT50_COOL_END_TEMP)/TS391LT50_COOL_DURATION);

const float TS391LT50[] = {
		TS391LT50_PREHEAT_START_TIME,
		TS391LT50_PREHEAT_DURATION,
		TS391LT50_PREHEAT_START_TEMP,
		TS391LT50_PREHEAT_END_TEMP,

		TS391LT50_SOAK_START_TIME,
		TS391LT50_SOAK_DURATION,
		TS391LT50_SOAK_START_TEMP,
		TS391LT50_SOAK_END_TEMP,

		TS391LT50_RAMP_START_TIME,
		TS391LT50_RAMP_DURATION,
		TS391LT50_RAMP_START_TEMP,
		TS391LT50_RAMP_END_TEMP,

		TS391LT50_REFLOW_START_TIME,
		TS391LT50_REFLOW_DURATION,
		TS391LT50_REFLOW_START_TEMP,
		TS391LT50_REFLOW_END_TEMP,

		TS391LT50_COOL_START_TIME,
		TS391LT50_COOL_DURATION,
		TS391LT50_COOL_END_TEMP,

		TS391LT50_PREHEAT_TEMP_INCREASE,
		TS391LT50_SOAK_TEMP_INCREASE,
		TS391LT50_RAMP_TEMP_INCREASE,
		TS391LT50_REFLOW_TEMP_INCREASE,
		TS391LT50_COOL_TEMP_DECREASE,
};

// 180 deg
const float SMD291AX50T3_PREHEAT_START_TIME  	=		0;
const float SMD291AX50T3_PREHEAT_DURATION		=		30.f;
const float SMD291AX50T3_PREHEAT_START_TEMP 	=		50.f;
const float SMD291AX50T3_PREHEAT_END_TEMP 		=		100.f;

const float SMD291AX50T3_SOAK_START_TIME 		=		SMD291AX50T3_PREHEAT_START_TIME+SMD291AX50T3_PREHEAT_DURATION+SAMPLE_TIME;
const float SMD291AX50T3_SOAK_DURATION 			=		90.f;
const float SMD291AX50T3_SOAK_START_TEMP 		=		SMD291AX50T3_PREHEAT_END_TEMP;
const float SMD291AX50T3_SOAK_END_TEMP 			=		150.f;

const float SMD291AX50T3_RAMP_START_TIME 		=		SMD291AX50T3_SOAK_START_TIME+SMD291AX50T3_SOAK_DURATION+SAMPLE_TIME;
const float SMD291AX50T3_RAMP_DURATION 			=		30.f;
const float SMD291AX50T3_RAMP_START_TEMP 		=		SMD291AX50T3_SOAK_END_TEMP;
const float SMD291AX50T3_RAMP_END_TEMP 			=		183.f;

const float SMD291AX50T3_REFLOW_START_TIME 		=		SMD291AX50T3_RAMP_START_TIME+SMD291AX50T3_RAMP_DURATION+SAMPLE_TIME;
const float SMD291AX50T3_REFLOW_DURATION		=		60.f;
const float SMD291AX50T3_REFLOW_START_TEMP 		=		SMD291AX50T3_RAMP_END_TEMP;
const float SMD291AX50T3_REFLOW_END_TEMP 		=		235.f;

const float SMD291AX50T3_COOL_START_TIME 		=		SMD291AX50T3_REFLOW_START_TIME + SMD291AX50T3_REFLOW_DURATION+SAMPLE_TIME;
const float SMD291AX50T3_COOL_DURATION			=		30.f;
const float SMD291AX50T3_COOL_END_TEMP 			=		183.f;

const float SMD291AX50T3_PREHEAT_TEMP_INCREASE 	=	((SMD291AX50T3_PREHEAT_END_TEMP-SMD291AX50T3_PREHEAT_START_TEMP)/SMD291AX50T3_PREHEAT_DURATION);
const float SMD291AX50T3_SOAK_TEMP_INCREASE 	= 	((SMD291AX50T3_SOAK_END_TEMP-SMD291AX50T3_SOAK_START_TEMP)/SMD291AX50T3_SOAK_DURATION);
const float SMD291AX50T3_RAMP_TEMP_INCREASE 	=	((SMD291AX50T3_RAMP_END_TEMP-SMD291AX50T3_RAMP_START_TEMP)/SMD291AX50T3_RAMP_DURATION);
const float SMD291AX50T3_REFLOW_TEMP_INCREASE 	=	((SMD291AX50T3_REFLOW_END_TEMP-SMD291AX50T3_REFLOW_START_TEMP)/SMD291AX50T3_RAMP_DURATION);
const float SMD291AX50T3_COOL_TEMP_DECREASE 	=	((SMD291AX50T3_REFLOW_END_TEMP-SMD291AX50T3_COOL_END_TEMP)/SMD291AX50T3_COOL_DURATION);

const float SMD291AX50T3[] = {
		SMD291AX50T3_PREHEAT_START_TIME,
		SMD291AX50T3_PREHEAT_DURATION,
		SMD291AX50T3_PREHEAT_START_TEMP,
		SMD291AX50T3_PREHEAT_END_TEMP,

		SMD291AX50T3_SOAK_START_TIME,
		SMD291AX50T3_SOAK_DURATION,
		SMD291AX50T3_SOAK_START_TEMP,
		SMD291AX50T3_SOAK_END_TEMP,

		SMD291AX50T3_RAMP_START_TIME,
		SMD291AX50T3_RAMP_DURATION,
		SMD291AX50T3_RAMP_START_TEMP,
		SMD291AX50T3_RAMP_END_TEMP,

		SMD291AX50T3_REFLOW_START_TIME,
		SMD291AX50T3_REFLOW_DURATION,
		SMD291AX50T3_REFLOW_START_TEMP,
		SMD291AX50T3_REFLOW_END_TEMP,

		SMD291AX50T3_COOL_START_TIME,
		SMD291AX50T3_COOL_DURATION,
		SMD291AX50T3_COOL_END_TEMP,

		SMD291AX50T3_PREHEAT_TEMP_INCREASE,
		SMD291AX50T3_SOAK_TEMP_INCREASE,
		SMD291AX50T3_RAMP_TEMP_INCREASE,
		SMD291AX50T3_REFLOW_TEMP_INCREASE,
		SMD291AX50T3_COOL_TEMP_DECREASE,
};
