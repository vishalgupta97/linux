
/*
This inline function was automatically generated using DecisionTreeToCpp Converter

It takes feature vector as single argument:
feature_vector[0] - Entry_Ratio
feature_vector[1] - RW_Ratio
feature_vector[2] - Core
feature_vector[3] - RPS
feature_vector[4] - table_komb_rwsem
feature_vector[5] - table_komb_rwsem_delegation
feature_vector[6] - table_percpu_rwsem
feature_vector[7] - table_rwsem


It returns a value.

Simply include this file to your project and use it
*/


inline int rwsem_decision_tree(long *feature_vector) 
{
	if (feature_vector[2] <= 54) {
		if (feature_vector[2] <= 12) {
			if (feature_vector[3] <= 37500) {
				if (feature_vector[2] <= 6) {
					if (feature_vector[3] <= 17500) {
						if (feature_vector[2] <= 3) {
							if (feature_vector[2] <= 2) {
								if (feature_vector[3] <= 8750) {
									return 6238;
								}
								else {
									return 9998;
								}
							}
							else {
								if (feature_vector[3] <= 6250) {
									return 9997;
								}
								else {
									return 17464;
								}
							}
						}
						else {
							if (feature_vector[3] <= 8750) {
								if (feature_vector[3] <= 6250) {
									return 19997;
								}
								else {
									return 29994;
								}
							}
							else {
								if (feature_vector[6] <= 0) {
									return 40000;
								}
								else {
									return 39990;
								}
							}
						}
					}
					else {
						if (feature_vector[2] <= 3) {
							if (feature_vector[2] <= 2) {
								if (feature_vector[6] <= 0) {
									return 24999;
								}
								else {
									return 24987;
								}
							}
							else {
								if (feature_vector[0] <= 2) {
									return 49974;
								}
								else {
									return 50001;
								}
							}
						}
						else {
							if (feature_vector[6] <= 0) {
								if (feature_vector[0] <= 6) {
									return 100000;
								}
								else {
									return 99999;
								}
							}
							else {
								if (feature_vector[1] <= 95) {
									return 99913;
								}
								else {
									return 100158;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[3] <= 17500) {
						if (feature_vector[3] <= 6250) {
							if (feature_vector[6] <= 0) {
								if (feature_vector[1] <= 15) {
									return 40001;
								}
								else {
									return 40001;
								}
							}
							else {
								if (feature_vector[1] <= 15) {
									return 40005;
								}
								else {
									return 39976;
								}
							}
						}
						else {
							if (feature_vector[3] <= 8750) {
								if (feature_vector[6] <= 0) {
									return 60001;
								}
								else {
									return 60015;
								}
							}
							else {
								if (feature_vector[6] <= 0) {
									return 80001;
								}
								else {
									return 80049;
								}
							}
						}
					}
					else {
						if (feature_vector[6] <= 0) {
							if (feature_vector[7] <= 0) {
								if (feature_vector[1] <= 3) {
									return 200001;
								}
								else {
									return 200000;
								}
							}
							else {
								if (feature_vector[0] <= 2) {
									return 199997;
								}
								else {
									return 199999;
								}
							}
						}
						else {
							if (feature_vector[1] <= 70) {
								if (feature_vector[1] <= 35) {
									return 200023;
								}
								else {
									return 192774;
								}
							}
							else {
								if (feature_vector[1] <= 95) {
									return 122941;
								}
								else {
									return 111320;
								}
							}
						}
					}
				}
			}
			else {
				if (feature_vector[2] <= 6) {
					if (feature_vector[2] <= 3) {
						if (feature_vector[2] <= 2) {
							if (feature_vector[3] <= 65000) {
								if (feature_vector[0] <= 6) {
									return 49999;
								}
								else {
									return 49987;
								}
							}
							else {
								if (feature_vector[6] <= 0) {
									return 79999;
								}
								else {
									return 79949;
								}
							}
						}
						else {
							if (feature_vector[3] <= 65000) {
								if (feature_vector[6] <= 0) {
									return 100000;
								}
								else {
									return 100014;
								}
							}
							else {
								if (feature_vector[6] <= 0) {
									return 160001;
								}
								else {
									return 154124;
								}
							}
						}
					}
					else {
						if (feature_vector[3] <= 65000) {
							if (feature_vector[6] <= 0) {
								if (feature_vector[0] <= 6) {
									return 200000;
								}
								else {
									return 199998;
								}
							}
							else {
								if (feature_vector[1] <= 70) {
									return 200032;
								}
								else {
									return 123371;
								}
							}
						}
						else {
							if (feature_vector[6] <= 0) {
								if (feature_vector[1] <= 35) {
									return 319998;
								}
								else {
									return 320011;
								}
							}
							else {
								if (feature_vector[1] <= 35) {
									return 320068;
								}
								else {
									return 156345;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[3] <= 65000) {
						if (feature_vector[6] <= 0) {
							if (feature_vector[4] <= 0) {
								if (feature_vector[5] <= 0) {
									return 399996;
								}
								else {
									return 400000;
								}
							}
							else {
								if (feature_vector[1] <= 15) {
									return 400002;
								}
								else {
									return 400012;
								}
							}
						}
						else {
							if (feature_vector[1] <= 35) {
								if (feature_vector[1] <= 15) {
									return 399960;
								}
								else {
									return 379155;
								}
							}
							else {
								if (feature_vector[1] <= 70) {
									return 191171;
								}
								else {
									return 116431;
								}
							}
						}
					}
					else {
						if (feature_vector[6] <= 0) {
							if (feature_vector[1] <= 0) {
								if (feature_vector[4] <= 0) {
									return 640096;
								}
								else {
									return 640000;
								}
							}
							else {
								if (feature_vector[0] <= 3) {
									return 639997;
								}
								else {
									return 640002;
								}
							}
						}
						else {
							if (feature_vector[1] <= 15) {
								if (feature_vector[1] <= 3) {
									return 639583;
								}
								else {
									return 641994;
								}
							}
							else {
								if (feature_vector[1] <= 35) {
									return 373557;
								}
								else {
									return 148966;
								}
							}
						}
					}
				}
			}
		}
		else {
			if (feature_vector[3] <= 37500) {
				if (feature_vector[3] <= 17500) {
					if (feature_vector[2] <= 22) {
						if (feature_vector[3] <= 6250) {
							if (feature_vector[1] <= 95) {
								if (feature_vector[1] <= 70) {
									return 80004;
								}
								else {
									return 80076;
								}
							}
							else {
								if (feature_vector[6] <= 0) {
									return 80002;
								}
								else {
									return 73325;
								}
							}
						}
						else {
							if (feature_vector[3] <= 8750) {
								if (feature_vector[1] <= 95) {
									return 119436;
								}
								else {
									return 108030;
								}
							}
							else {
								if (feature_vector[6] <= 0) {
									return 160002;
								}
								else {
									return 140172;
								}
							}
						}
					}
					else {
						if (feature_vector[3] <= 6250) {
							if (feature_vector[6] <= 0) {
								if (feature_vector[1] <= 35) {
									return 135005;
								}
								else {
									return 135004;
								}
							}
							else {
								if (feature_vector[1] <= 70) {
									return 130719;
								}
								else {
									return 69141;
								}
							}
						}
						else {
							if (feature_vector[3] <= 8750) {
								if (feature_vector[6] <= 0) {
									return 202504;
								}
								else {
									return 156090;
								}
							}
							else {
								if (feature_vector[6] <= 0) {
									return 270003;
								}
								else {
									return 195196;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[2] <= 22) {
						if (feature_vector[6] <= 0) {
							if (feature_vector[1] <= 70) {
								if (feature_vector[7] <= 0) {
									return 400001;
								}
								else {
									return 399998;
								}
							}
							else {
								if (feature_vector[5] <= 0) {
									return 400001;
								}
								else {
									return 400052;
								}
							}
						}
						else {
							if (feature_vector[1] <= 35) {
								if (feature_vector[1] <= 15) {
									return 400257;
								}
								else {
									return 375692;
								}
							}
							else {
								if (feature_vector[1] <= 70) {
									return 190426;
								}
								else {
									return 98071;
								}
							}
						}
					}
					else {
						if (feature_vector[6] <= 0) {
							if (feature_vector[1] <= 95) {
								if (feature_vector[4] <= 0) {
									return 675007;
								}
								else {
									return 675002;
								}
							}
							else {
								if (feature_vector[4] <= 0) {
									return 675001;
								}
								else {
									return 675096;
								}
							}
						}
						else {
							if (feature_vector[1] <= 15) {
								if (feature_vector[1] <= 8) {
									return 675152;
								}
								else {
									return 656354;
								}
							}
							else {
								if (feature_vector[1] <= 35) {
									return 377095;
								}
								else {
									return 82193;
								}
							}
						}
					}
				}
			}
			else {
				if (feature_vector[6] <= 0) {
					if (feature_vector[2] <= 22) {
						if (feature_vector[3] <= 65000) {
							if (feature_vector[0] <= 2) {
								if (feature_vector[1] <= 95) {
									return 800000;
								}
								else {
									return 800011;
								}
							}
							else {
								if (feature_vector[1] <= 15) {
									return 800018;
								}
								else {
									return 800054;
								}
							}
						}
						else {
							if (feature_vector[1] <= 15) {
								if (feature_vector[0] <= 3) {
									return 1280072;
								}
								else {
									return 1280013;
								}
							}
							else {
								if (feature_vector[0] <= 3) {
									return 1280047;
								}
								else {
									return 1280390;
								}
							}
						}
					}
					else {
						if (feature_vector[3] <= 65000) {
							if (feature_vector[1] <= 70) {
								if (feature_vector[7] <= 0) {
									return 1350004;
								}
								else {
									return 1350139;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 1350022;
								}
								else {
									return 1346969;
								}
							}
						}
						else {
							if (feature_vector[7] <= 0) {
								if (feature_vector[0] <= 6) {
									return 2160128;
								}
								else {
									return 2160487;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 1976965;
								}
								else {
									return 1561143;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[1] <= 8) {
						if (feature_vector[2] <= 22) {
							if (feature_vector[3] <= 65000) {
								if (feature_vector[1] <= 3) {
									return 800063;
								}
								else {
									return 800798;
								}
							}
							else {
								if (feature_vector[1] <= 3) {
									return 1280123;
								}
								else {
									return 1231184;
								}
							}
						}
						else {
							if (feature_vector[3] <= 65000) {
								if (feature_vector[1] <= 3) {
									return 1350092;
								}
								else {
									return 1215362;
								}
							}
							else {
								if (feature_vector[1] <= 3) {
									return 2160590;
								}
								else {
									return 1222116;
								}
							}
						}
					}
					else {
						if (feature_vector[1] <= 35) {
							if (feature_vector[1] <= 15) {
								if (feature_vector[0] <= 6) {
									return 674586;
								}
								else {
									return 641748;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 376714;
								}
								else {
									return 364270;
								}
							}
						}
						else {
							if (feature_vector[1] <= 70) {
								if (feature_vector[2] <= 22) {
									return 190478;
								}
								else {
									return 111662;
								}
							}
							else {
								if (feature_vector[2] <= 22) {
									return 100879;
								}
								else {
									return 69926;
								}
							}
						}
					}
				}
			}
		}
	}
	else {
		if (feature_vector[3] <= 17500) {
			if (feature_vector[6] <= 0) {
				if (feature_vector[2] <= 122) {
					if (feature_vector[3] <= 6250) {
						if (feature_vector[7] <= 0) {
							if (feature_vector[0] <= 3) {
								if (feature_vector[4] <= 0) {
									return 405013;
								}
								else {
									return 405015;
								}
							}
							else {
								if (feature_vector[4] <= 0) {
									return 405013;
								}
								else {
									return 405011;
								}
							}
						}
						else {
							if (feature_vector[0] <= 6) {
								if (feature_vector[0] <= 2) {
									return 405013;
								}
								else {
									return 405032;
								}
							}
							else {
								if (feature_vector[1] <= 35) {
									return 405027;
								}
								else {
									return 405145;
								}
							}
						}
					}
					else {
						if (feature_vector[3] <= 8750) {
							if (feature_vector[0] <= 6) {
								if (feature_vector[0] <= 3) {
									return 607515;
								}
								else {
									return 607554;
								}
							}
							else {
								if (feature_vector[7] <= 0) {
									return 607553;
								}
								else {
									return 600238;
								}
							}
						}
						else {
							if (feature_vector[7] <= 0) {
								if (feature_vector[1] <= 70) {
									return 810062;
								}
								else {
									return 790210;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 781273;
								}
								else {
									return 654497;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[3] <= 6250) {
						if (feature_vector[2] <= 189) {
							if (feature_vector[7] <= 0) {
								if (feature_vector[0] <= 6) {
									return 810037;
								}
								else {
									return 782937;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 763411;
								}
								else {
									return 537221;
								}
							}
						}
						else {
							if (feature_vector[7] <= 0) {
								if (feature_vector[0] <= 6) {
									return 1072593;
								}
								else {
									return 986413;
								}
							}
							else {
								if (feature_vector[0] <= 3) {
									return 970022;
								}
								else {
									return 692729;
								}
							}
						}
					}
					else {
						if (feature_vector[7] <= 0) {
							if (feature_vector[3] <= 8750) {
								if (feature_vector[2] <= 189) {
									return 1168204;
								}
								else {
									return 1484087;
								}
							}
							else {
								if (feature_vector[1] <= 35) {
									return 1837686;
								}
								else {
									return 1414260;
								}
							}
						}
						else {
							if (feature_vector[1] <= 0) {
								if (feature_vector[2] <= 189) {
									return 1446500;
								}
								else {
									return 1928606;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 951767;
								}
								else {
									return 506899;
								}
							}
						}
					}
				}
			}
			else {
				if (feature_vector[1] <= 3) {
					if (feature_vector[1] <= 0) {
						if (feature_vector[2] <= 122) {
							if (feature_vector[3] <= 6250) {
								if (feature_vector[0] <= 6) {
									return 405022;
								}
								else {
									return 405024;
								}
							}
							else {
								if (feature_vector[3] <= 8750) {
									return 607525;
								}
								else {
									return 810012;
								}
							}
						}
						else {
							if (feature_vector[3] <= 6250) {
								if (feature_vector[2] <= 189) {
									return 810045;
								}
								else {
									return 1080060;
								}
							}
							else {
								if (feature_vector[2] <= 189) {
									return 1446463;
								}
								else {
									return 1890050;
								}
							}
						}
					}
					else {
						if (feature_vector[2] <= 122) {
							if (feature_vector[3] <= 8750) {
								if (feature_vector[3] <= 6250) {
									return 405042;
								}
								else {
									return 607584;
								}
							}
							else {
								if (feature_vector[0] <= 3) {
									return 810046;
								}
								else {
									return 809994;
								}
							}
						}
						else {
							if (feature_vector[0] <= 3) {
								if (feature_vector[2] <= 189) {
									return 533628;
								}
								else {
									return 511751;
								}
							}
							else {
								if (feature_vector[3] <= 6250) {
									return 517993;
								}
								else {
									return 500316;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[1] <= 8) {
						if (feature_vector[2] <= 122) {
							if (feature_vector[3] <= 6250) {
								if (feature_vector[0] <= 2) {
									return 405282;
								}
								else {
									return 405868;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 466515;
								}
								else {
									return 452725;
								}
							}
						}
						else {
							if (feature_vector[0] <= 3) {
								if (feature_vector[2] <= 189) {
									return 114743;
								}
								else {
									return 112603;
								}
							}
							else {
								if (feature_vector[2] <= 189) {
									return 111445;
								}
								else {
									return 108866;
								}
							}
						}
					}
					else {
						if (feature_vector[1] <= 15) {
							if (feature_vector[2] <= 122) {
								if (feature_vector[3] <= 6250) {
									return 175207;
								}
								else {
									return 126411;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 59676;
								}
								else {
									return 57681;
								}
							}
						}
						else {
							if (feature_vector[1] <= 35) {
								if (feature_vector[2] <= 122) {
									return 34190;
								}
								else {
									return 33399;
								}
							}
							else {
								if (feature_vector[1] <= 70) {
									return 16285;
								}
								else {
									return 9641;
								}
							}
						}
					}
				}
			}
		}
		else {
			if (feature_vector[1] <= 0) {
				if (feature_vector[3] <= 37500) {
					if (feature_vector[2] <= 122) {
						if (feature_vector[5] <= 0) {
							if (feature_vector[6] <= 0) {
								if (feature_vector[0] <= 2) {
									return 2025004;
								}
								else {
									return 2025010;
								}
							}
							else {
								if (feature_vector[0] <= 3) {
									return 2025020;
								}
								else {
									return 2025032;
								}
							}
						}
						else {
							if (feature_vector[0] <= 6) {
								if (feature_vector[0] <= 2) {
									return 2025008;
								}
								else {
									return 2025225;
								}
							}
							else {
								return 2025005;
							}
						}
					}
					else {
						if (feature_vector[2] <= 189) {
							if (feature_vector[0] <= 2) {
								if (feature_vector[4] <= 0) {
									return 4050209;
								}
								else {
									return 4051768;
								}
							}
							else {
								if (feature_vector[5] <= 0) {
									return 4050030;
								}
								else {
									return 4050204;
								}
							}
						}
						else {
							if (feature_vector[7] <= 0) {
								if (feature_vector[5] <= 0) {
									return 5400051;
								}
								else {
									return 5400270;
								}
							}
							else {
								if (feature_vector[0] <= 5) {
									return 5402051;
								}
								else {
									return 5407579;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[2] <= 122) {
						if (feature_vector[3] <= 65000) {
							if (feature_vector[0] <= 2) {
								if (feature_vector[4] <= 0) {
									return 4050024;
								}
								else {
									return 4056059;
								}
							}
							else {
								if (feature_vector[4] <= 0) {
									return 4050039;
								}
								else {
									return 4050103;
								}
							}
						}
						else {
							if (feature_vector[7] <= 0) {
								if (feature_vector[0] <= 2) {
									return 6485009;
								}
								else {
									return 6480113;
								}
							}
							else {
								if (feature_vector[0] <= 3) {
									return 5826481;
								}
								else {
									return 4372933;
								}
							}
						}
					}
					else {
						if (feature_vector[6] <= 0) {
							if (feature_vector[7] <= 0) {
								if (feature_vector[2] <= 189) {
									return 8297455;
								}
								else {
									return 7819942;
								}
							}
							else {
								if (feature_vector[2] <= 189) {
									return 4707486;
								}
								else {
									return 6086384;
								}
							}
						}
						else {
							if (feature_vector[3] <= 65000) {
								if (feature_vector[2] <= 189) {
									return 8100091;
								}
								else {
									return 10800122;
								}
							}
							else {
								if (feature_vector[2] <= 189) {
									return 12960152;
								}
								else {
									return 17280178;
								}
							}
						}
					}
				}
			}
			else {
				if (feature_vector[6] <= 0) {
					if (feature_vector[7] <= 0) {
						if (feature_vector[1] <= 8) {
							if (feature_vector[3] <= 37500) {
								if (feature_vector[2] <= 122) {
									return 2025168;
								}
								else {
									return 4319470;
								}
							}
							else {
								if (feature_vector[1] <= 3) {
									return 6013803;
								}
								else {
									return 4298338;
								}
							}
						}
						else {
							if (feature_vector[1] <= 95) {
								if (feature_vector[1] <= 35) {
									return 2594534;
								}
								else {
									return 1304919;
								}
							}
							else {
								if (feature_vector[3] <= 37500) {
									return 3684850;
								}
								else {
									return 6181905;
								}
							}
						}
					}
					else {
						if (feature_vector[0] <= 3) {
							if (feature_vector[1] <= 8) {
								if (feature_vector[1] <= 3) {
									return 516267;
								}
								else {
									return 839495;
								}
							}
							else {
								if (feature_vector[0] <= 2) {
									return 1321870;
								}
								else {
									return 1093860;
								}
							}
						}
						else {
							if (feature_vector[0] <= 6) {
								if (feature_vector[1] <= 3) {
									return 487062;
								}
								else {
									return 801121;
								}
							}
							else {
								if (feature_vector[2] <= 122) {
									return 599567;
								}
								else {
									return 495814;
								}
							}
						}
					}
				}
				else {
					if (feature_vector[1] <= 3) {
						if (feature_vector[2] <= 122) {
							if (feature_vector[3] <= 37500) {
								if (feature_vector[0] <= 3) {
									return 2028734;
								}
								else {
									return 2037318;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 2132566;
								}
								else {
									return 2053538;
								}
							}
						}
						else {
							if (feature_vector[0] <= 3) {
								if (feature_vector[2] <= 189) {
									return 531742;
								}
								else {
									return 519780;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 509529;
								}
								else {
									return 497512;
								}
							}
						}
					}
					else {
						if (feature_vector[1] <= 8) {
							if (feature_vector[2] <= 122) {
								if (feature_vector[0] <= 6) {
									return 456195;
								}
								else {
									return 442339;
								}
							}
							else {
								if (feature_vector[0] <= 6) {
									return 113511;
								}
								else {
									return 107789;
								}
							}
						}
						else {
							if (feature_vector[1] <= 15) {
								if (feature_vector[2] <= 122) {
									return 102505;
								}
								else {
									return 58826;
								}
							}
							else {
								if (feature_vector[1] <= 35) {
									return 33479;
								}
								else {
									return 11714;
								}
							}
						}
					}
				}
			}
		}
	}
}