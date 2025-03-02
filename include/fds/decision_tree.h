
/*
This inline function was automatically generated using DecisionTreeToCpp Converter

It takes feature vector as single argument:
feature_vector[0] - Entry_Ratio
feature_vector[1] - Core
feature_vector[2] - RPS
feature_vector[3] - table_aqs
feature_vector[4] - table_cna
feature_vector[5] - table_komb
feature_vector[6] - table_komb_delegation
feature_vector[7] - table_spinlock


It returns a value.

Simply include this file to your project and use it
*/


inline int decision_tree(long *feature_vector) 
{
	if (feature_vector[1] <= 22) {
		if (feature_vector[2] <= 300000) {
			if (feature_vector[2] <= 37500) {
				if (feature_vector[1] <= 12) {
					if (feature_vector[1] <= 6) {
						return 26971;
					}
					else {
						return 96524;
					}
				}
				else {
					if (feature_vector[2] <= 17500) {
						return 118037;
					}
					else {
						return 400006;
					}
				}
			}
			else {
				if (feature_vector[1] <= 12) {
					if (feature_vector[1] <= 6) {
						return 180715;
					}
					else {
						return 611496;
					}
				}
				else {
					if (feature_vector[2] <= 65000) {
						return 800162;
					}
					else {
						return 1360917;
					}
				}
			}
		}
		else {
			if (feature_vector[1] <= 3) {
				if (feature_vector[1] <= 2) {
					if (feature_vector[3] <= 0) {
						return 499992;
					}
					else {
						return 500058;
					}
				}
				else {
					if (feature_vector[0] <= 12) {
						return 1000070;
					}
					else {
						return 1000627;
					}
				}
			}
			else {
				if (feature_vector[0] <= 6) {
					if (feature_vector[5] <= 0) {
						return 2269042;
					}
					else {
						return 3416362;
					}
				}
				else {
					if (feature_vector[5] <= 0) {
						return 1178741;
					}
					else {
						return 2562000;
					}
				}
			}
		}
	}
	else {
		if (feature_vector[2] <= 17500) {
			if (feature_vector[1] <= 122) {
				if (feature_vector[1] <= 54) {
					if (feature_vector[2] <= 8750) {
						return 168005;
					}
					else {
						return 270016;
					}
				}
				else {
					if (feature_vector[2] <= 6250) {
						return 388255;
					}
					else {
						return 601147;
					}
				}
			}
			else {
				if (feature_vector[5] <= 0) {
					if (feature_vector[6] <= 0) {
						return 601858;
					}
					else {
						return 1390352;
					}
				}
				else {
					if (feature_vector[2] <= 6250) {
						return 945273;
					}
					else {
						return 1675908;
					}
				}
			}
		}
		else {
			if (feature_vector[6] <= 0) {
				if (feature_vector[5] <= 0) {
					if (feature_vector[1] <= 54) {
						return 1449727;
					}
					else {
						return 656538;
					}
				}
				else {
					if (feature_vector[1] <= 54) {
						return 2072119;
					}
					else {
						return 4300585;
					}
				}
			}
			else {
				if (feature_vector[1] <= 54) {
					if (feature_vector[2] <= 65000) {
						return 1060784;
					}
					else {
						return 2485441;
					}
				}
				else {
					if (feature_vector[0] <= 12) {
						return 5140845;
					}
					else {
						return 3228773;
					}
				}
			}
		}
	}
}