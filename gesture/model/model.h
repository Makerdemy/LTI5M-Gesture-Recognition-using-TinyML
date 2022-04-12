#ifdef INCLUDED_BY_NEUTON_C

/* Model info */
#define NEUTON_MODEL_HEADER_VERSION 1
#define NEUTON_MODEL_QLEVEL 32
#define NEUTON_MODEL_FLOAT_SUPPORT 1
#define NEUTON_MODEL_TASK_TYPE 1  // binary classification
#define NEUTON_MODEL_NEURONS_COUNT 5
#define NEUTON_MODEL_WEIGHTS_COUNT 21
#define NEUTON_MODEL_INPUTS_COUNT 300
#define NEUTON_MODEL_INPUT_LIMITS_COUNT 300
#define NEUTON_MODEL_OUTPUTS_COUNT 2
#define NEUTON_MODEL_LOG_SCALE_OUTPUTS 0

/* Preprocessing */
#define NEUTON_PREPROCESSING_ENABLED 0
#define NEUTON_BITMASK_ENABLED 0

/* Limits */
static const float modelInputMin[] = {
	-23.32, -3.8789999, -21.361, -0.89300001, -4.3660002, -2.1600001, -24.311001,
	-4.7690001, -21.639, -1, -4.3660002, -1.934, -23.118, -6.105, -22.98, -1.385,
	-4.3660002, -1.737, -21.524, -6.8909998, -22.830999, -1.985, -4.3660002,
	-1.097, -19.532, -7.1059999, -24.138, -2.3050001, -4.3660002, -0.37400001,
	-17.094999, -6.2729998, -27.122, -1.523, -4.3660002, -0.164, -15.165, -5.4640002,
	-28.663, -1.582, -4.3660002, -0.375, -14.437, -5.77, -27.533001, -1.226,
	-4.3660002, -0.74000001, -12.804, -5.6170001, -26.049, -1.189, -4.3660002,
	-0.72399998, -11.933, -5.1329999, -24.339001, -1.137, -3.4990001, -0.77999997,
	-9.5959997, -4.52, -25.187, -1.59, -2.885, -0.833, -7.2020001, -4.5149999,
	-25.622999, -2.339, -2.6010001, -0.71499997, -5.473, -4.415, -25.412001,
	-1.842, -1.846, -0.73699999, -3.845, -3.6300001, -25.042999, -1.4, -1.587,
	-0.57999998, -2.701, -3.0450001, -24.205, -1.094, -1.906, -0.72899997,
	-2.332, -2.447, -23.679001, -1.728, -1.872, -0.83999997, -2.734, -2.48,
	-23.219, -1.83, -1.658, -0.80900002, -3.5769999, -2.552, -22.922001, -1.752,
	-1.739, -1.033, -4.1799998, -2.8540001, -22.806999, -1.813, -1.378, -1.126,
	-4.4530001, -2.6619999, -22.606001, -2.1099999, -1.073, -1.301, -5.4439998,
	-2.1070001, -21.687, -2.342, -1.0089999, -1.473, -6.158, -2.428, -20.356001,
	-2.0799999, -0.91799998, -1.3099999, -6.7129998, -3.6340001, -19.077, -2.174,
	-1.255, -1.1390001, -6.9000001, -3.7449999, -18.474001, -2.1960001, -1.791,
	-1.151, -6.79, -3.0599999, -18.076, -1.951, -1.72, -1.561, -6.546, -4.9660001,
	-17.502001, -1.852, -1.224, -1.626, -6.2199998, -7.3790002, -17.32, -2.125,
	-0.79299998, -1.728, -5.7459998, -7.8390002, -17.98, -2.112, -0.80400002,
	-1.706, -5.6069999, -5.684, -18.34, -1.979, -0.70099998, -1.804, -5.2820001,
	-4.171, -17.899, -1.944, -0.26300001, -2.168, -4.875, -4.0170002, -17.070999,
	-2.063, -0.079999998, -1.7180001, -4.5149999, -4.0029998, -16.198999, -1.995,
	-0.257, -1.433, -3.9360001, -3.6630001, -15.572, -1.826, -0.382, -1.41,
	-3.256, -1.6900001, -15.107, -2.2309999, -0.29899999, -1.46, -3.1989999,
	-1.475, -14.341, -2.7290001, -0.153, -1.443, -4.0409999, -1.2309999, -13.575,
	-2.454, -0.117, -1.464, -5.066, -1.331, -12.699, -2.6570001, -0.097000003,
	-1.426, -5.454, -1.427, -11.77, -2.4560001, -0.24699999, -1.3710001, -4.9219999,
	-1.427, -10.396, -1.773, -0.25, -1.37, -3.467, -1.1440001, -8.6330004,
	-1.617, -0.27900001, -1.3710001, -3.0739999, -1.369, -7.652, -2.4170001,
	-1.1210001, -1.5140001, -2.8440001, -1.408, -6.8379998, -2.744, -1.517,
	-1.427, -2.6670001, -1.279, -5.7750001, -2.4489999, -1.7869999, -1.501,
	-2.4130001, -1.322, -5.152, -2.0179999, -1.744, -1.7309999, -3.7490001,
	-1.2309999, -4.0939999, -2.6459999, -1.29, -1.804, -5.2820001, -1.14, -3.7349999,
	-2.3889999, -1.772, -1.574, -5.928, -0.991, -2.8399999, -1.728, -2.135,
	-1.567, -5.0949998, -0.96700001, -1.911, -1.893, -1.919, -1.622, -4.2709999,
	-0.93900001, -3.098, -2.3499999, -1.308, -1.962, -4.9180002, -0.852, -2.6289999,
	-1.7869999, -1.54, -1.585 };
static const float modelInputMax[] = {
	2.418, 4.3189998, 18.056999, 3.3469999, 2.855, 3.6289999, 2.931, 9.0410004,
	19.604, 2.977, 2.48, 3.105, 2.9779999, 10.238, 17.593, 4.2329998, 1.495,
	2.3510001, 2.9159999, 6.8429999, 13.111, 4.3660002, 0.52899998, 2.125,
	5.3439999, 4.539, 10.434, 4.3660002, 1.133, 2.421, 8.0109997, 5.2670002,
	9.2510004, 4.3660002, 1.668, 2.322, 9.5480003, 5.1479998, 8.9449997, 4.3660002,
	2.0840001, 2.0510001, 10.142, 4.8270001, 8.8059998, 4.3660002, 2.3080001,
	1.965, 10.06, 4.697, 8.7010002, 4.3660002, 2.5350001, 2.1630001, 9.29,
	4.6830001, 8.5810003, 3.6789999, 2.9449999, 1.7079999, 8.4420004, 4.7639999,
	8.5039997, 3.5929999, 3.5250001, 1.784, 7.7620001, 4.9180002, 8.2650003,
	2.72, 4.0450001, 1.799, 7.2690001, 5.033, 8.1400003, 1.273, 4.3629999,
	1.671, 7.178, 5.1430001, 8.4469995, 0.48800001, 4.3660002, 1.397, 7.3210001,
	4.994, 9.151, 0.85799998, 4.3660002, 1.1440001, 7.4790001, 4.6399999, 9.8549995,
	1.341, 4.3660002, 1.043, 7.6609998, 4.3569999, 9.7489996, 1.0039999, 4.3660002,
	1.105, 7.8769999, 4.1560001, 9.3950005, 0.75199997, 4.3660002, 1.048, 8.0109997,
	3.994, 9.3179998, 0.84100002, 4.3660002, 1.087, 8.1499996, 3.9030001, 9.9499998,
	1.039, 4.3660002, 1.013, 9.2609997, 3.859, 9.9549999, 1.3279999, 4.3660002,
	0.81, 10.118, 3.73, 10.056, 1.272, 4.3660002, 0.62, 10.444, 3.9170001,
	10.151, 1.473, 4.3660002, 0.59100002, 10.089, 4.171, 10.4, 1.591, 4.3660002,
	0.86500001, 9.0880003, 4.415, 10.927, 1.929, 4.3660002, 1.801, 9.0120001,
	4.6160002, 11.147, 2.346, 4.3660002, 2.7119999, 10.444, 4.664, 12.139,
	2.253, 4.3660002, 2.2190001, 10.697, 4.6259999, 12.886, 2.1659999, 4.3660002,
	1.429, 8.2259998, 4.434, 13.235, 1.775, 4.3660002, 2.026, 7.8429999, 4.3239999,
	13.245, 2.0350001, 4.3660002, 1.845, 7.8579998, 4.2379999, 13.704, 1.8890001,
	4.3660002, 1.1390001, 8.0010004, 5.4679999, 12.775, 1.65, 4.3660002, 1.0039999,
	8.1499996, 7.0819998, 12, 1.276, 4.3660002, 0.62699997, 8.2889996, 6.7319999,
	11.899, 1.6, 4.3660002, 0.19599999, 8.3170004, 5.4640002, 12.349, 1.28,
	4.3660002, 0.19599999, 8.2460003, 4.908, 14.619, 0.69400001, 4.3660002,
	0.52399999, 8.2220001, 3.9890001, 17.885, 0.583, 4.3660002, 0.65899998,
	8.1120005, 4.1609998, 19.474001, 0.69300002, 4.3660002, 0.72799999, 8.1070004,
	4.1849999, 19.268999, 1.3150001, 4.3660002, 0.94800001, 7.9489999, 4.5539999,
	18.382999, 1.229, 4.3660002, 1.1059999, 7.8870001, 4.4959998, 14.815, 0.77499998,
	4.3660002, 0.87199998, 7.757, 4.75, 16.98, 0.43700001, 4.3660002, 0.48899999,
	9.0690002, 5.4109998, 17.497, 0.41100001, 4.3660002, 0.472, 10.314, 5.473,
	16.597, 0.75800002, 4.3660002, 0.40099999, 10.755, 5.9660001, 16.290001,
	1.109, 4.3660002, 0.49399999, 11.473, 5.9569998, 16.643999, 1.33, 4.3660002,
	0.55699998, 11.186, 6.0289998, 19.393, 1.345, 4.3660002, 0.75, 10.415,
	6.2540002, 19.666, 1.354, 4.3660002, 0.76599997, 10.511, 5.8179998, 18.086,
	1.395, 4.3660002, 1.189, 11.291, 5.2290001, 19.743, 1.4529999, 4.3660002,
	1.21 };

static const float modelOutputMin[] = { 0, 0 };
static const float modelOutputMax[] = { 1, 1 };

/* Types */
typedef float coeff_t;
typedef float weight_t;
typedef double acc_signed_t;
typedef double acc_unsigned_t;
typedef uint16_t sources_size_t;
typedef uint8_t weights_size_t;
typedef uint8_t neurons_size_t;

/* Structure */
static const weight_t modelWeights[] = {
	0.39596373, -0.99994898, -1, 0.99209636, -0.067610115, -1, -1, 0.5, -1,
	1, 0.5, -1, 0.5, -1, -1, 1, 1, 0, 1, -1, -0.16616403 };

static const sources_size_t modelLinks[] = {
	47, 58, 172, 260, 300, 127, 138, 140, 154, 218, 300, 1, 300, 99, 153, 250,
	252, 300, 0, 3, 300 };

static const weights_size_t modelIntLinksBoundaries[] = { 0, 5, 12, 13, 20 };
static const weights_size_t modelExtLinksBoundaries[] = { 5, 11, 13, 18, 21 };

static const coeff_t modelFuncCoeffs[] = {
	40, 20.050001, 40, 10.075001, 34.320564 };

static const neurons_size_t modelOutputNeurons[] = { 4, 2 };

#endif // INCLUDED_BY_NEUTON_C

