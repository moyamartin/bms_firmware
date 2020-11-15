#ifndef _BATTERY_MODEL_H
#define _BATTERY_MODEL_H

#define SAMPLE_SOCS 55

const float32_t soc_lut[55] = {
	0.998550201961916, 0.995643392960185, 0.989808136180035, 0.978188113098673,
	0.946252885170225, 0.943320830929025, 0.937510819388344, 0.925898009232545,
	0.893959174841316, 0.891041546451241, 0.885227928447778, 0.873496105020196,
	0.789375360646278, 0.786468551644547, 0.780629688401616, 0.769009665320254,
	0.684784333525678, 0.681881130986728, 0.676042267743797, 0.664415031736873,
	0.580193306405078, 0.577290103866128, 0.571476485862666, 0.559860069244085,
	0.475605885747259, 0.472673831506059, 0.466867426428159, 0.455247403346798,
	0.371014858626659, 0.368111656087709, 0.362298038084247, 0.350667195614541,
	0.266434650894403, 0.263502596653203, 0.257692585112522, 0.246068955568379,
	0.214144547028275, 0.211237738026544, 0.205398874783612, 0.193778851702250,
	0.161843623773803, 0.158911569532602, 0.153101557991922, 0.141485141373341,
	0.109549913444893, 0.106614252740912, 0.100800634737450, 0.089177005193306,
	0.087795729948067, 0.057256203115983, 0.054349394114252, 0.048539382573571,
	0.046678447778419, 0.004966099249855, 0.002034045008655};

const float32_t F_lut[55][9] = {
	{0.824499520069884, 0, 0, 0, 0.997193705430708, 0, 0, 0, 1},
	{0.729143543326006, 0, 0, 0, 0.996774027596312, 0, 0, 0, 1},
	{0.613493258599889, 0, 0, 0, 0.996627082432468, 0, 0, 0, 1},
	{0.508028517884498, 0, 0, 0, 0.996154647860267, 0, 0, 0, 1},
	{0.681603381893132, 0, 0, 0, 0.996907420910721, 0, 0, 0, 1},
	{0.678177306030184, 0, 0, 0, 0.996577558304987, 0, 0, 0, 1},
	{0.563818888346490, 0, 0, 0, 0.996255089670588, 0, 0, 0, 1},
	{0.849114357892530, 0, 0, 0, 0.996732795050363, 0, 0, 0, 1},
	{0.880218894552132, 0, 0, 0, 0.99668685958970,  0, 0, 0, 1},
	{0.875242823437144, 0, 0, 0, 0.996647253132177, 0, 0, 0, 1},
	{0.770443906258385, 0, 0, 0, 0.996621748241788, 0, 0, 0, 1},
	{0.881066218963118, 0, 0, 0, 0.997367400271149, 0, 0, 0, 1},
	{0.923818863936941, 0, 0, 0, 0.997437711280810, 0, 0, 0, 1},
	{0.920506982633287, 0, 0, 0, 0.997374829207253, 0, 0, 0, 1},
	{0.871195676768168, 0, 0, 0, 0.997500047247692, 0, 0, 0, 1},
	{0.887701071880676, 0, 0, 0, 0.998138106582012, 0, 0, 0, 1},
	{0.890737377441570, 0, 0, 0, 0.997750892193852, 0, 0, 0, 1},
	{0.889408123722429, 0, 0, 0, 0.997786930493099, 0, 0, 0, 1},
	{0.837206370256018, 0, 0, 0, 0.998176704459620, 0, 0, 0, 1},
	{0.905184153409707, 0, 0, 0, 0.998749900476944, 0, 0, 0, 1},
	{0.950144267740838, 0, 0, 0, 0.998749900476944, 0, 0, 0, 1},
	{0.948012569334725, 0, 0, 0, 0.998306159068758, 0, 0, 0, 1},
	{0.933458497517139, 0, 0, 0, 0.998744527918644, 0, 0, 0, 1},
	{0.919543619054510, 0, 0, 0, 0.998727144711811, 0, 0, 0, 1},
	{0.919627363746559, 0, 0, 0, 0.998249731382813, 0, 0, 0, 1},
	{0.912107177355594, 0, 0, 0, 0.997773802343023, 0, 0, 0, 1},
	{0.901380668693759, 0, 0, 0, 0.997439921781423, 0, 0, 0, 1},
	{0.903760514566670, 0, 0, 0, 0.997090730363302, 0, 0, 0, 1},
	{0.642152341380241, 0, 0, 0, 0.997574998818111, 0, 0, 0, 1},
	{0.598666752000509, 0, 0, 0, 0.997550661880110, 0, 0, 0, 1}
	{0.495499359036353, 0, 0, 0, 0.997394974263073, 0, 0, 0, 1},
	{0.893130484059498, 0, 0, 0, 0.997484538600240, 0, 0, 0, 1},
	{0.724829344500521, 0, 0, 0, 0.998692703401530, 0, 0, 0, 1},
	{0.686080367153810, 0, 0, 0, 0.998376957023828, 0, 0, 0, 1},
	{0.625797134042177, 0, 0, 0, 0.997863418952908, 0, 0, 0, 1},
	{0.907294154164313, 0, 0, 0, 0.997805979707757, 0, 0, 0, 1},
	{0.923883778258424, 0, 0, 0, 0.998229177253619, 0, 0, 0, 1}, 
	{0.925096547348489  0, 0, 0, 0.998228488988736, 0, 0, 0, 1}, 
	{0.924966676945597	0, 0, 0, 0.997932057318591, 0, 0, 0, 1}, 		 
	{0.915193145344427	0, 0, 0, 0.997582357145831, 0, 0, 0, 1}, 			 
	{0.921815518420733 	0, 0, 0, 0.998061846921042, 0, 0, 0, 1}, 			 
	{0.917843764127121	0, 0, 0, 0.997977493738314, 0, 0, 0, 1}, 			 
	{0.891267370282904	0, 0, 0, 0.997704904254253, 0, 0, 0, 1}, 			 
	{0.917849942088673	0, 0, 0, 0.997367656934433, 0, 0, 0, 1}, 			 
	{0.927212301887824	0, 0, 0, 0.998043055434520, 0, 0, 0, 1}, 			 
	{0.923918199422554	0, 0, 0, 0.997892260013875, 0, 0, 0, 1}, 			 
	{0.706843733793562	0, 0, 0, 0.996102418694646, 0, 0, 0, 1}, 			 
	{0.900046227963568	0, 0, 0, 0.995955929954623, 0, 0, 0, 1}, 			 
	{0.660050076020646	0, 0, 0, 0.880779051172843, 0, 0, 0, 1}, 			 
	{0.842102720913321	0, 0, 0, 0.997750367342481, 0, 0, 0, 1}, 			 
	{0.886446520422366	0, 0, 0, 0.997264144938528, 0, 0, 0, 1}, 			 
	{0.930979202162406	0, 0, 0, 0.997410481754020, 0, 0, 0, 1}, 			 
	{0.769488855715429	0, 0, 0, 0.336214674125174, 0, 0, 0, 1}, 			 
	{0.950310773516501	0, 0, 0, 0.999191001697624, 0, 0, 0, 1},            
	{0.959244666010665	0, 0, 0, 0.998394845596616, 0, 0, 0, 1}
}

const float32_t G_lut[55][3] = {
	{0.002378204454078, 0.000069883586930, 0.0000095785440613}, 
	{0.003689497332514, 0.000070655066759, 0.0000095785440613}, 
	{0.005593128769518, 0.000069890014708, 0.0000095785440613},
	{0.004990429311732, 0.000068560686578, 0.0000095785440613},
	{0.004476474523060, 0.000007293238143, 0.0000095785440613},
	{0.004611007069407, 0.000073249400418, 0.0000095785440613},
	{0.005983413512282, 0.000076514585307, 0.0000095785440613},
	{0.000687778311095, 0.000063802037699, 0.0000095785440613},
	{0.001101062676631, 0.000074622015611, 0.0000095785440613},
	{0.001177205309410, 0.000075602326189, 0.0000095785440613},
	{0.002312649391541, 0.000079421571660, 0.0000095785440613},
	{0.000566718626431, 0.000063127100217, 0.0000095785440613},
	{0.000653396096282, 0.000078351533257, 0.0000095785440613},
	{0.000658993619122, 0.000080184565038, 0.0000095785440613},
	{0.000956904680330, 0.000078419608717, 0.0000095785440613},
	{0.000594341081242, 0.000058999647772, 0.0000095785440613},
	{0.00106323952714,  0.000084679734291, 0.0000095785440613},
	{0.00103916979030,  0.000085328464124, 0.0000095785440613},
	{0.00144503343254,  0.000077402705434, 0.0000095785440613},
	{0.000530514676487, 0.000053710257705, 0.0000095785440613},
	{0.000502009169613, 0.000053710257705, 0.0000095785440613},
	{0.000506117637015, 0.000072775512070, 0.0000095785440613},
	{0.000500134243760, 0.000053941088520, 0.0000095785440613},
	{0.000500145034743, 0.000043700321707, 0.0000095785440613},
	{0.000500148273404, 0.000055137812262, 0.0000095785440613},
	{0.000590260960192, 0.000049827472259, 0.0000095785440613},
	{0.000676089240431, 0.000049747695684, 0.0000095785440613},
	{0.000500306874521, 0.000049896880287, 0.0000095785440613},
	{0.00370074903695,  0.000063122502979, 0.0000095785440613},
	{0.00475673321542,  0.000060388529587, 0.0000095785440613},
	{0.00717534020279,  0.000060013540856, 0.0000095785440613},
	{0.000551727278603, 0.000054445322670, 0.0000095785440613},
	{0.00292198170186,  0.000063433992837, 0.0000095785440613},
	{0.00405763234990,  0.000056829664588, 0.0000095785440613},
	{0.00510777529098,  0.000058281677699, 0.0000095785440613},
	{0.00052350896830,  0.000054000851880, 0.0000095785440613},
	{0.000500005017427, 0.000057199487261, 0.0000095785440613},
	{0.000517200475955, 0.000052644601252, 0.0000095785440613},
	{0.000524439938042, 0.000051508652483, 0.0000095785440613},
	{0.000500000108039, 0.000052863561549, 0.0000095785440613},
	{0.000615216237788, 0.000056877925885, 0.0000095785440613},
	{0.000676417037096, 0.000056934983766, 0.0000095785440613},
	{0.00105590064730,  0.000057822897693, 0.0000095785440613},
	{0.000617595316294, 0.000052856021035, 0.0000095785440613},
	{0.000819897362362, 0.000059414567121, 0.0000095785440613},
	{0.000949176650849, 0.000058505836266, 0.0000095785440613},
	{0.00852481832362,  0.000081864003114, 0.0000095785440613},
	{0.00234001950705,  0.000075675256261, 0.0000095785440613},
	{0.00764075430570,  0.000000001724848, 0.0000095785440613},
	{0.00747440490465,  0.000000001233326, 0.0000095785440613},
	{0.00590911224392,  0.000000001259087, 0.0000095785440613},
	{0.00417388383455,  0.000000001117157, 0.0000095785440613},
	{0.00807158551797,  0.000080370932342, 0.0000095785440613},
	{0.00522306158339,  0.000000001042400, 0.0000095785440613},
	{0.00479714628947,  0.000000001150456, 0.0000095785440613}
}

const float32_t H_lut[3] = {-1, -1, 0.890700000000000}

const float32_t D_lut[55] = {
	-0.0285738051664718, -0.0276430909419455, -0.0251536944405590, 
	-0.0270174274049931, -0.0223561291553651, -0.0224661872706683,
	-0.0219372756242863, -0.0300203125111896, -0.0262519194887284,
	-0.0263571289877418, -0.0243294830934685, -0.0288261248263196,
	-0.0264605668387840, -0.0265411979071445, -0.0259812733590617,
	-0.0272821765819163, -0.0243324826289660, -0.0246580282274638,
	-0.0241536784005057, -0.0270806403055194, -0.0250212733364972,
	-0.0255222480720633, -0.0267773073276219, -0.0266428405483400,
	-0.0252496410333513, -0.0255497096076879, -0.0252750580176866,
	-0.0266699639776231, -0.0210016120348710, -0.0197396234049825,
	-0.0176182870106548, -0.0271900094921629, -0.0219094012617613,
	-0.0204571641867893, -0.0198963106934741, -0.0284958080921751,
	-0.0283177989824512, -0.0288506503375844, -0.0293209801550658,
	-0.0305110159569864, -0.0307176374652452, -0.0312770995106585,
	-0.0307797010090205, -0.0343250022213900, -0.0369438252243108, 
	-0.0384962468932097, -0.0234383861539077, -0.0346812492514071,
	-0.0284102988471330, -0.0282991017281021, -0.0318657939223064,
	-0.0330583799896668, -0.0343304532583233, -0.0421897198855773,
	-0.0410529883871751};

#endif 
