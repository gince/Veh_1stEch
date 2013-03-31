// vehicle routing only
#include <ilcplex/ilocplex.h>
#include <set>
#include <algorithm>
#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>

ILOSTLBEGIN

#include "functions.h"

using namespace std;

int main() {
#include "definitions.h"
	try {
		
		const char* datafile  = "data.txt";
		ifstream dfile(datafile);
		if ( !dfile ) {
			cerr << "ERROR: could not open file '" << datafile
			<< "' for reading" << endl;
			throw (-1);
		}
		
		dfile >> P >> Vin >> tF >> tB >> w >> beta >> W >> V;
		
		//converts hours to periods
		for (i = 0; i < M; i++) {
			for (j = 0; j < N; j++) {
				tF[i][j] = ceil((double)tF[i][j]/(24/H));
				tB[j][i] = ceil((double)tB[j][i]/(24/H));
//				cout << tF[i][j] << endl;
//				cout << tB[j][i] << endl;
			}
		}
		
		//converts priorities to a total of 3
		double totalPriority;
		for (k = 0; k < K; k++) {
			totalPriority += beta[k] ; 
		}
		cout << "totalPriority = " << totalPriority << endl ;
		for (k = 0; k < K; k++) {
			beta[k] = 3 * beta[k] / totalPriority ;
			cout << "beta[k] = " << beta[k] << endl ;
		}
		
		vector<DNode> districts = getDNodes(); // AVC, BAH, BAK, ..., SIL;
		vector<SNode> suppliers = getSNodes();
		vector<Cluster> clusters = getClusters();
		
		int tP = 0;
		for (j = 0; j < N; j++)
			tP += P[j];
		
		double minCommWeight = w[0];
		for (k = 1; k < K; k++) {
			if (w[k] < minCommWeight)
				minCommWeight = w[k];
		}
		cout << "minCommWeight = " << minCommWeight << endl;
		
		double maxPop = P[0];
		for (j = 1; j < N; j++) {
			if (P[j] > maxPop)
				maxPop = P[j];
		}
		cout << "maxPop = " << maxPop << endl;
		
		int xBound = W / minCommWeight ;
		cout << "xBound = " << xBound << endl;
		
		vT = IloIntVarArray(env, T, 0, tP);
		
		for (t = 1; t < T; t++) {
			v[t] = IntVarMatrix(env, H);
			for (h=0; h < H; h++) {
				v[t][h] = IloIntVarArray(env, M, 0, tP);
			}
		}
		
		for (t = 0; t < T; t++) {
			g[t] = IntVarMatrix(env, H);
			rho[t] = IntVarMatrix(env, H);
			for (h=0; h < H; h++) {
				g[t][h] = IloIntVarArray(env, M, 0, tP);
				rho[t][h] = IloIntVarArray(env, N, 0, tP);
			}
		}
		
		for (t = 1; t < T; t++) {
			y[t] = IntVar3dMatrix(env, H);
			gama[t] = IntVar3dMatrix(env, H);
			x[t] = NumVar4dMatrix(env, H);
			vSF[t] = IntVarMatrix(env, N);
			for (h = 0; h < H; h++) {
				y[t][h] = IntVarMatrix(env, M);
				gama[t][h] = IntVarMatrix(env, N);
				x[t][h] = NumVar3dMatrix(env, M);
				for(i=0; i < M; i++) {
					y[t][h][i] = IloIntVarArray(env, N, 0, tP);
					x[t][h][i] = NumVarMatrix(env, N);
					for(j=0; j < N; j++) {
						x[t][h][i][j] = IloNumVarArray(env, K, 0, tP);
					}
				}
				for(j=0; j < N; j++) {
					gama[t][h][j] = IloIntVarArray(env, M, 0, tP);
				}
			}
			for(j=0; j < N; j++) {
				vSF[t][j] = IloIntVarArray(env, K, 0, tP);
			}
		}
		
		//OBJECTIVE0 :: Minimize total shortfall in vehicle capacity
		IloExpr tvSF(env);
		for (t = 1; t < T; t++)
			for (j = 0; j < N; j++)
				for (k = 0; k < K; k++)
					tvSF += beta[k] * vSF[t][j][k];
		
		IloExpr tvB(env);
		for (t = 1; t < T; t++)
			for (h = 0; h < H; h++)
				for (j = 0; j < N; j++)
					for (i = 0; i < M; i++)
						tvB += gama[t][h][j][i];
		
		IloExpr tvF(env);
		for (t = 1; t < T; t++)
			for (h = 0; h < H; h++)
				for (j = 0; j < N; j++)
					for (i = 0; i < M; i++)
						tvF += y[t][h][i][j];
		
		vector<IloObjective> objs;
		vector<IloExpr> Z;
		Z.push_back(tvSF);
		Z.push_back(tV);
		Z.push_back(tvB);
		Z.push_back(tvF);
		
		IloObjective minSF(env, Z[0], IloObjective::Minimize);
		IloObjective mintV(env, Z[1], IloObjective::Minimize);
		IloObjective maxtvB(env, Z[2], IloObjective::Maximize);
		IloObjective maxtvF(env, Z[3], IloObjective::Maximize);
		objs.push_back(minSF);
		objs.push_back(mintV);
		objs.push_back(maxtvB);
		objs.push_back(maxtvF);
		
		//Adding objectives to env
		mod.add(objs[0]);
		
		cout << "RESTRICTIONS" << endl;
		// to capture  min #vehicles for 0 shortfall
		//				mod.add(tvSF == 0);
/*		
		for (t = 1; t < T; t++) {
			for (i = 0; i < M; i++) {
				for (j = 0; j < N; j++) {
					IloExpr yH(env);
					for (h = 0; h < H; h++) {
						yH += y[t][h][i][j];
					}
					mod.add(yH <= 50) ;
					yH.end();
				}
			}
		}
		for (t = 1; t < T; t++) {
			for (j = 0; j < N; j++) {
				IloExpr yIH(env);
				for (i = 0; i < M; i++) {
					for (h = 0; h < H; h++) {
						yIH += y[t][h][i][j];
					}
				}
//				mod.add(yIH <= 50) ;
				yIH.end();
			}
		}
 */
				// vT[t] is fixed
		mod.add(vT[1] == 21);
		mod.add(vT[2] == 20);
		mod.add(vT[3] == 20);
		mod.add(vT[4] == 20);
		mod.add(vT[5] == 20);
		
		//VEHICLE CONSTRAINTS
		cout << "CONSTRAINT V-1" << endl;
		IloExpr vTT(env);
		for (t = 1; t < T; t++)	{
			vTT += vT[t];
		}
//		mod.add(vTT <= tV);
//		mod.add(vTT <= V);
		vTT.end();
		
		cout << "CONSTRAINT V-2" << endl;
		for (t = 1; t < T; t++) {
			IloExpr vITI(env);
			for (i = 0; i < M; i++)
				vITI += v[t][0][i];
			mod.add(vITI <= vT[t]);
			vITI.end();
		}
		
		cout << "CONSTRAINT V-2-EK silinebilir mi bak" << endl;
		for (t = 1; t < T; t++)
			for (h = 1; h < H; h++)
				for (i = 0; i < M; i++)
					mod.add(v[t][h][i] == 0);
		
		cout << "CONSTRAINT V-3" << endl;
		for (i = 0; i < M; i++)
			for (h = 0; h < H; h++) {
				if (h == H - 1) {
					mod.add(g[0][h][i] == Vin[i]);
				} else {
					mod.add(g[0][h][i] == 0);
				}
			}
		
		cout << "CONSTRAINT V-4" << endl;
		for (j = 0; j < N; j++)
			for (h = 0; h < H; h++) {
					mod.add(rho[0][h][j] == 0);
			}
		
		cout << "CONSTRAINT V-5" << endl;
		for (t = 1; t < T; t++) {
			for (i = 0; i < M; i++) {
				for (h = 0; h < H; h++) {
					IloExpr yJ(env);
					IloExpr gamaJ(env);
					for (j = 0; j < N; j++) {
						yJ += y[t][h][i][j];
						div_t div1 = div ((int)h - (int)tB[j][i], (int)H);
						int shpHr = div1.rem ;
						if (shpHr < 0) {
							shpHr += H ;
						}
						int shpDay = floor ((double)((int)h - (int)tB[j][i]) / H) ;
						if (t + shpDay > 0) {
							gamaJ += gama[t + shpDay][shpHr][j][i];
						}
					}
					int invHr = h - 1;
					if (invHr < 0) {
						invHr += H;
					}
					int invDay = floor ((double) (h - 1) / H) ;
					mod.add(g[t + invDay][invHr][i] + v[t][h][i] + gamaJ - g[t][h][i] - yJ == 0);
					yJ.end();
					gamaJ.end();
				}
			}
		}
		
		cout << "CONSTRAINT V-6" << endl;
		for (t = 1; t < T; t++) {
			for (j = 0; j < N; j++) {
				for (h = 0; h < H; h++) {
					IloExpr yI(env);
					IloExpr gamaI(env);
					for (i = 0; i < M; i++) {
						gamaI += gama[t][h][j][i];
						div_t div1 = div ((int)h - (int)tF[i][j], (int)H);
						int shpHr = div1.rem ;
						if (shpHr < 0) {
							shpHr += H ;
						}
						int shpDay = floor ((double)((int)h - (int)tF[i][j]) / H) ;
						if (t + shpDay > 0) {
							yI += y[t + shpDay][shpHr][i][j];
						}
					}
					int invHr = h - 1;
					if (invHr < 0) {
						invHr += H;
					}
					int invDay = floor ((double) (h - 1) / H) ;
					mod.add(rho[t + invDay][invHr][j] + yI - rho[t][h][j] - gamaI == 0);
					yI.end();
					gamaI.end();
				}
			}
		}

		cout << "CONSTRAINT V-7" << endl ;
		for (t = 1; t < T; t++) {
			for (h = 0; h < H; h++) {
				for (i = 0; i < M; i++) {
					for (j = 0; j < N; j++) {
						IloExpr xK(env) ;
						for (k = 0; k < K; k++) {
							xK += w[k] * x[t][h][i][j][k] ;
						}
						mod.add(W * y[t][h][i][j] - xK >= 0) ;
						xK.end() ;
					}
				}
			}
		}
		
		cout << "CONSTRAINT V-8" << endl;
		for (t = 1; t < T; t++) {
			for (j = 0; j < N; j++) {
				for (k = 0; k < K; k++) {
					IloExpr xIH(env) ;
					for (h = 0; h < H; h++) {
						for (i = 0; i < M; i++) {
							xIH += x[t][h][i][j][k] ;
						}
					}
					mod.add(xIH + vSF[t][j][k] == P[j]) ;
					xIH.end() ;
				}
			}
		}		
		
		cout << "CONSTRAINT SON" << endl ;
				
		cout << "SOME EXPRESSIONS TO CAPTURE" << endl ;
		IloExpr y0HI0(env) ; 
		for (h = 0; h < H; h++) {
			for(i = 0; i < M; i++){
				y0HI0 += y[1][h][i][0] ;
			}			
		}
		
		IloExpr vTotal(env);
		for (t = 1; t < T; t++) {
			for (h = 0; h < H; h++)
				for (i = 0; i < M; i++)
					vTotal += v[t][h][i];
		}
		
		IloCplex cplex(env);
		
		// OPTIMALITY GAP 0.20%
		cplex.setParam(IloCplex::EpGap, 0.002);
//		cplex.setParam(IloCplex::RootAlg, IloCplex::Network);
		cplex.setParam(IloCplex::MIPEmphasis, 2);
		cplex.setParam(IloCplex::Symmetry, 5);
		cplex.extract(mod);
		cplex.exportModel("model.lp");
		
		cplex.solve();
		
		cplex.out() << "solution status = " << cplex.getStatus() << endl;
		cplex.out() << "objective value = " << cplex.getObjValue() << endl;
		
		cout << "u[t][j][k]-------------------" << endl;
		int totalSF = 0 ; 
		for(t = 1; t < T; t++){
			for(j = 0; j < N; j++){
				for(k = 0; k < K; k++){
					if (cplex.getValue(vSF[t][j][k]) > 0) {
						totalSF += cplex.getValue(vSF[t][j][k]) ; 
						cout << "[t,j,k] = [" << t << "," << j + 1 << "," << k + 1 << "] > " << cplex.getValue(vSF[t][j][k]) << "\t" << endl;
					}
				}
			}
		}
		cout << "Total shortfall = " << totalSF << endl;
		
		cout << "# Vehicles = " << cplex.getValue(vTotal) << endl;
		
		cout << "initial veh. inv.--------------" << endl;
		for(i = 0; i < M; i++){
			cout << "i = " << i << " > " << Vin[i] << "\t" << endl;
		}
		
		cout << "vT[t]-------------------" << endl;
		for(t = 1; t < T; t++){
			if (cplex.getValue(vT[t]) > 0) {
				cout << "t = " << t << " > " << cplex.getValue(vT[t]) << "\t" << endl;
			}
		}
		
		cout << "v[t][h][i]-------------------" << endl;
		for(t = 1; t < T; t++){
			for(h = 0; h < H; h++){
				for (i = 0; i < M; i++) {
					if (cplex.getValue(v[t][h][i]) > 0) {
						cout << "[t,h,i] = [" << t << "," << h << "," << i + 1 << "] > " << cplex.getValue(v[t][h][i]) << "\t" << endl;
					}
				}
			}
		}
		
		cout << "total y[1][h][i][0]-------------------" << endl ;
		cout << "total flow to j = 0 at t = 1 > " << cplex.getValue(y0HI0) << endl ;
		
		cout << endl ;
		cout << "g[t][h][i]-------------------" << endl ;
		for (t = 0; t < 2; t++) {
			for (h = 0; h < H; h++) {
				for (i = 0; i < M; i++) {
				if (cplex.getValue(g[t][h][i]) > 0)
					cout << "[t,h,i] = [" << t << "," << h << "," << i + 1 << "] > " << cplex.getValue(g[t][h][i]) << endl ;																				
				}
			}
		}

		
		cout << endl ;
		cout << "y[1][h][i][1]: day = 1, vehicles to j = 1 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(y[1][h][i][0]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(y[1][h][i][0]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "y[1][h][i][1]: day = 1, vehicles to j = 2 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(y[1][h][i][1]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(y[1][h][i][1]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "y[1][h][i][1]: day = 1, vehicles to j = 3 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(y[1][h][i][2]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(y[1][h][i][2]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "y[1][h][i][1]: day = 1, vehicles to j = 4 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(y[1][h][i][3]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(y[1][h][i][3]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "y[1][h][i][1]: day = 1, vehicles to j = 5 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(y[1][h][i][4]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(y[1][h][i][4]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "y[1][h][1][j]: day = 1, vehicles from i = 1 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (j = 0; j < N; j++) {
				if (cplex.getValue(y[1][h][0][j]) > 0)
					cout << "[h,j,t_ij] = [" << h << "," << j + 1 << "," << tF[0][j] << "] > " << cplex.getValue(y[1][h][0][j]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "y[1][h][1][j]: day = 1, vehicles from i = 2 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (j = 0; j < N; j++) {
				if (cplex.getValue(y[1][h][1][j]) > 0)
					cout << "[h,j,t_ij] = [" << h << "," << j + 1 << "," << tF[1][j] << "] > " << cplex.getValue(y[1][h][1][j]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "y[1][h][1][j]: day = 1, vehicles from i = 3 ------" << endl ;
		for (h = 0; h < H; h++) {
			for (j = 0; j < N; j++) {
				if (cplex.getValue(y[1][h][2][j]) > 0)
					cout << "[h,j,t_ij] = [" << h << "," << j + 1 << "," << tF[2][j] << "] > " << cplex.getValue(y[1][h][2][j]) << endl ;					
			}
		}
		
		cout << endl ;
		for (t = 1; t < T; t++) {
			cout << "Total flow from i, day = " << t << " ------" << endl ;
			for (i = 0; i < M; i++) {
				cout << " i = "<< i + 1 << "-------" << endl ;
				for (j = 0; j < N; j++) {
					IloInt tVfromItoJ;
					for (h = 0; h < H; h++) {
						if (h + tF[i][j] < H)
							tVfromItoJ += cplex.getValue(y[t][h][i][j]);
					}
					if (tVfromItoJ > 0)
						cout << "[day,i,j] = [1," << i + 1 << "," << j + 1 << "]> " << tVfromItoJ << endl ;
					tVfromItoJ = 0;
				}
			}
		}		
		
		cout << endl ;
		for (t = 1; t < T; t++) {
			cout << "Total flow from j, day = " << t << " ------" << endl ;
			for (j = 0; j < N; j++) {
				cout << " j = "<< j + 1 << "-------" << endl ;
				for (i = 0; i < M; i++) {
					IloInt tVfromJtoI;
					for (h = 0; h < H; h++) {
						if (h + tB[j][i] < H)
							tVfromJtoI += cplex.getValue(gama[t][h][j][i]);
					}
					if (tVfromJtoI > 0)
						cout << "[day,j,i] = [1," << j + 1 << "," << i + 1 << "]> " << tVfromJtoI << endl ;
					tVfromJtoI = 0;
				}
			}
		}		
		
		cout << endl ;
		cout << "y[1][h][3][j]-------------------" << endl ;
		for (h = 0; h < H; h++) {
			for (j = 0; j < N; j++) {
				if (cplex.getValue(y[1][h][2][j]) > 0)
					cout << "[h,j] = [" << h << "," << j + 1 << "] > " << cplex.getValue(y[1][h][2][j]) << endl ;																				
			}
		}
		
		cout << endl ;
		cout << "gama[1][h][1][i]-------------------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(gama[1][h][0][i]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(gama[1][h][0][i]) << endl ;																				
			}
		}
		cout << "gama[2][h][1][i]-------------------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(gama[2][h][0][i]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(gama[2][h][0][i]) << endl ;																				
			}
		}
		/*		
		cout << endl ;
		cout << "x[1][h][i][0][0]-------------------" << endl ;
		//		for (t = 1; t < T; t++) {
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				for (k = 0; k < K; k++) {
					if (cplex.getValue(x[1][h][i][0][k]) > 0)
						cout << "[h,i,k] = [" << h << "," << i << "," << k << "] > " << cplex.getValue(x[1][h][i][0][k]) << endl ;																				
				}
			}
		}
		*/
/*		 
		 cplex.out() << "objective value = " << cplex.getObjValue() << endl;
		 cout << "min shortfall = " << cplex.getValue(tvSF) << endl;
		 cout << "min # vehicles = " << cplex.getValue(tV) << endl;
		 cout << "tD = " << tD << endl;
		 */		
		/*		d1.end(); d2.end();	s.end(); sT.end(); I.end();	x.end(); Sk.end(); J.end();
		 E.end(); zeta.end(); X.end(); p.end(); m.end();	v.end(); vT.end(); g.end();
		 gama.end(); y.end(); rho.end(); env.end(); mod.end(); tvSF.end(); vTotal.end();
		 
		 */		
	}
	catch (IloException& e) {
		cerr << "ERROR: " << e.getMessage() << endl;
	}
	catch (...) {
		cerr << "Error" << endl;
	}
	return 0;
}
