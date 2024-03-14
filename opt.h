#ifndef OPT_H
#define OPT_H

#include <avr/pgmspace.h>

#define C12880MA_CHANELS 250

#define EXPOSURE_TIME_SEL_100us     0
#define EXPOSURE_TIME_SEL_200us     1
#define EXPOSURE_TIME_SEL_500us     2
#define EXPOSURE_TIME_SEL_1ms       3
#define EXPOSURE_TIME_SEL_2ms       4
#define EXPOSURE_TIME_SEL_5ms       5
#define EXPOSURE_TIME_SEL_10ms      6
#define EXPOSURE_TIME_SEL_20ms      7
#define EXPOSURE_TIME_SEL_50ms      8
#define EXPOSURE_TIME_SEL_100ms     9
#define EXPOSURE_TIME_SEL_500ms     10
#define EXPOSURE_TIME_SEL_1s        11
#define EXPOSURE_TIME_SEL_N         10
#define EXPOSURE_TIME_100us         52L
#define EXPOSURE_TIME_200us         152L
#define EXPOSURE_TIME_500us         452L
#define EXPOSURE_TIME_1ms           952L
#define EXPOSURE_TIME_2ms           1952L
#define EXPOSURE_TIME_5ms           4952L
#define EXPOSURE_TIME_10ms          9952L
#define EXPOSURE_TIME_20ms          19952L
#define EXPOSURE_TIME_50ms          44952L
#define EXPOSURE_TIME_100ms         99952L
#define EXPOSURE_TIME_500ms         449952L
#define EXPOSURE_TIME_1s            999952L

#define CP150
// #define CP160

const PROGMEM float unitcoeff[] = {0.0001,0.0002,0.0005,0.001,0.002,0.005,0.01,0.02,0.05,0.1};
#ifdef CP150
const PROGMEM int wl_10nm[] = {330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720,730,740,750,760,770,780,790,800};
const PROGMEM int wl_1nm[] = {330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,797,798,799,800};
#endif
#ifdef CP160
const PROGMEM int wl_10nm[] = {310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720,730,740,750,760,770,780,790,800};
const PROGMEM int wl_1nm[] = {310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,797,798,799,800};
#endif

const PROGMEM char model[] = "CP150";
const PROGMEM char name[] = "UVVISSPEC";
const PROGMEM char serialnumber[] = "CP15022G02689";
const PROGMEM float wccoeff[] = {313.46735852581,2.68150101597628,-8.24352325657611E-04,-9.87714042605041E-06,1.57733091864455E-08,-1.99571645127481E-12};
const PROGMEM float spectralsensitivitycoeff[] = {9198200.45361354,9198200.45361354,9198200.45361354,9198200.45361354,9198200.45361354,9198200.45361354,9198200.45361354,9198200.45361354,9198200.45361354,9372878.88617757,9670262.46865962,10019374.553139,10393302.6755051,10760561.1589297,11090998.528958,11374837.7739309,11592170.6028443,11670578.3804005,11619417.0736543,11507272.8846368,11356413.8647858,11163806.7130372,10978562.9936576,10845996.1347148,10802226.7112986,10986929.3804634,11293431.5697834,11701893.413299,12206921.7813185,12837133.6336354,13503515.6565488,14168796.0705796,14780235.8908895,15283707.2002241,15705352.1770945,16031785.0135722,16220484.6844377,16259385.818307,16203638.1130492,16068180.8302215,15852010.9094442,15578491.8932956,15287521.117507,15005188.3858105,14769194.9021782,14595378.3412981,14494865.8351976,14485458.160482,14671221.5201349,14984307.377453,15358203.3453362,15762495.4813551,16094187.3087319,16401872.6196138,16716899.2971705,17047105.1199472,17457349.1689031,17891839.0949465,18314010.4626268,18703580.3033897,18981039.9553907,19191871.4303459,19362518.8774408,19502207.9584245,19684095.6963479,19851002.9408879,19957756.9552329,19981599.7036187,19802930.2724836,19486916.5197723,19109385.2871025,18699957.1277823,18321772.3053681,17981677.5657326,17663491.5373484,17375054.6682708,17133913.067423,16946119.2889293,16799984.4866257,16695523.6411845,16640412.444429,16642979.4001013,16675437.8386965,16728787.2516815,16793726.1795336,16857432.898712,16917636.6706434,16967537.5190587,17000429.6222543,16980234.0156271,16933837.9054056,16872903.8110383,16802843.7662319,16734194.8715536,16669708.4710573,16607061.2007857,16548309.7041331,16497177.9206929,16459670.8890027,16428937.9102488,16404397.1238399,16385474.54224,16370331.7427963,16359816.2737496,16354136.7681748,16353226.701871,16356622.3326452,16364179.2357215,16376732.519758,16394513.6166079,16417749.5739251,16457725.5900303,16501269.0298503,16543866.9359609,16582145.4754791,16609844.3042039,16622705.4417763,16624555.838158,16614434.3866799,16591397.6794557,16543264.7265409,16480440.1745524,16409312.9148412,16333007.4365008,16257291.9929372,16199463.5234291,16139030.4211669,16073311.4400997,15999672.8094982,15909177.7867,15798283.7344803,15677673.2986732,15548913.3948987,15413544.3181863,15277880.6079866,15140256.2224708,14997229.0189078,14848290.2747518,14692941.5478258,14513488.5323393,14325814.1638531,14140378.508881,13962241.9736023,13796369.4351714,13689215.213924,13601724.952477,13518243.9496109,13431489.105306,13334316.7026494,13171997.9077342,12987016.1056864,12796211.0476731,12606281.8046859,12423797.1030534,12283089.3036643,12166022.3410054,12060213.2358548,11963328.826839,11873080.7197817,11776608.1822272,11676415.0107971,11582783.9327139,11497635.3067794,11422849.2905242,11370650.3875213,11354931.1267023,11345225.0216203,11335970.7840825,11321721.2541925,11297142.390453,11219081.6111326,11125279.4344248,11022256.7434973,10913329.7361234,10801746.2877855,10699005.7043118,10608092.1419557,10518623.2164114,10430116.9541691,10342101.4855553,10254114.9433057,10150191.2451605,10046250.5400124,9947376.58101648,9856397.12869448,9776077.45774187,9716745.43564674,9704554.90948624,9702126.27363632,9704835.36332807,9708158.2239876,9707669.98242826,9687375.82927071,9637344.73252471,9578300.63713812,9511438.39795833,9437927.83439542,9358914.01841706,9281711.67781136,9208305.91661945,9129253.52154566,9043304.15021161,8949236.80954931,8845859.47193905,8722045.07852476,8570141.25058263,8411613.17474652,8249691.29387806,8087533.49024485,7928226.04595844,7778178.47453875,7658492.87959348,7546021.89696362,7439794.85124721,7338862.15574073,7242295.02742015,7149185.20197964,7054180.5557738,6960088.16507604,6868355.54013827,6778972.310518,6691927.94834673,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,6607211.77432048,};
const PROGMEM int wl_lut_1nm[] = {6,6,6,7,7,8,8,8,9,9,9,10,10,11,11,11,12,12,12,13,13,14,14,14,15,15,15,16,16,17,17,17,18,18,18,19,19,20,20,20,21,21,22,22,22,23,23,23,24,24,25,25,25,26,26,26,27,27,28,28,28,29,29,30,30,30,31,31,31,32,32,33,33,33,34,34,35,35,35,36,36,36,37,37,38,38,38,39,39,40,40,40,41,41,42,42,42,43,43,43,44,44,45,45,45,46,46,47,47,47,48,48,49,49,49,50,50,51,51,51,52,52,53,53,53,54,54,55,55,55,56,56,57,57,57,58,58,59,59,59,60,60,61,61,61,62,62,63,63,63,64,64,65,65,65,66,66,67,67,67,68,68,69,69,69,70,70,71,71,72,72,72,73,73,74,74,74,75,75,76,76,76,77,77,78,78,79,79,79,80,80,81,81,81,82,82,83,83,84,84,84,85,85,86,86,87,87,87,88,88,89,89,90,90,90,91,91,92,92,93,93,93,94,94,95,95,96,96,96,97,97,98,98,99,99,99,100,100,101,101,102,102,103,103,103,104,104,105,105,106,106,107,107,107,108,108,109,109,110,110,111,111,111,112,112,113,113,114,114,115,115,116,116,116,117,117,118,118,119,119,120,120,121,121,122,122,123,123,123,124,124,125,125,126,126,127,127,128,128,129,129,130,130,131,131,131,132,132,133,133,134,134,135,135,136,136,137,137,138,138,139,139,140,140,141,141,142,142,143,143,144,144,145,145,146,146,147,147,148,148,149,149,150,150,151,151,152,152,153,153,154,154,155,155,156,156,157,157,158,158,159,159,160,161,161,162,162,163,163,164,164,165,165,166,166,167,167,168,168,169,170,170,171,171,172,172,173,173,174,174,175,176,176,177,177,178,178,179,179,180,181,181,182,182,183,183,184,185,185,186,186,187,187,188,189,189,190,190,191,192,192,193,193,194,194,195,196,196,197,197,198,199,199,200,200,201,202,202,203,204,204,205,205,206,207,207,208,208,209,210,210,211,212,212,213,214,214,215,215,216,217,217,218,219,219,220,221,221,222,223,223,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};

#endif