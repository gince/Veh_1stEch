// vehicle routing only
#include <ilcplex/ilocplex.h>
#include <set>
#include <vector>
#include <iostream>
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
		
		dfile >> P >> Vin >> tF >> tB >> w >> W >> V;
		
		//converts hours to periods
		for (i = 0; i < M; i++) {
			for (j = 0; j < N; j++) {
				tF[i][j] = ceil((double)tF[i][j]/6);
				tB[j][i] = ceil((double)tB[j][i]/6);
			}
		}
		
		vector<DNode> districts = getDNodes(); // AVC, BAH, BAK, ..., SIL;
		vector<SNode> suppliers = getSNodes();
		vector<Cluster> clusters = getClusters();
		
		int tP = 0;
		for (j = 0; j < N; j++)
			tP += P[j];
		
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
					tvSF += vSF[t][j][k];
		
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
		mod.add(objs[1]);
		
		cout << "RESTRICTIONS" << endl;
		// to capture  min #vehicles for 0 shortfall
		mod.add(tvSF == 0);
		
		//VEHICLE CONSTRAINTS
		cout << "CONSTRAINT V-1" << endl;
		IloExpr vTT(env);
		for (t = 1; t < T; t++)	{
			vTT += vT[t];
		}
		mod.add(vTT <= tV);
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
				if (h == 3) {
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
						div_t div1 = div ((int)h - (int)tB[j][i], 4);
						int shpHr = div1.rem ;
						if (shpHr < 0) {
							shpHr += 4 ;
						}
						int shpDay = floor ((double)((int)h - (int)tB[j][i]) / 4) ;
//						cout << "[t,h,t_ji] = [" << t << "," << h << "," << tB[j][i] << "] > " << "shpDay = " << t + shpDay << endl ;
						if (t + shpDay > 0) {
							gamaJ += gama[t + shpDay][shpHr][j][i];
						}
					}
					int invHr = h - 1;
					if (invHr < 0) {
						invHr += 4;
					}
					int invDay = floor ((double) (h - 1) / 4) ;
//					cout << " [t,h] = [" << t << "," << h << "] > " << "invDay = " << t + invDay << endl ;
//					cout << " [t,h] = [" << t << "," << h << "] > " << "invHr = " << invHr << endl ;
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
						div_t div1 = div ((int)h - (int)tF[i][j], 4);
						int shpHr = div1.rem ;
						if (shpHr < 0) {
							shpHr += 4 ;
						}
						int shpDay = floor ((double)((int)h - (int)tF[i][j]) / 4) ;
//						cout << "[t,h,t_ij] = [" << t << "," << h << "," << tF[i][j] << "] > " << "shpDay = " << t + shpDay << endl ;
						if (t + shpDay > 0) {
							yI += y[t + shpDay][shpHr][i][j];
						}
					}
					int invHr = h - 1;
					if (invHr < 0) {
						invHr += 4;
					}
					int invDay = floor ((double) (h - 1) / 4) ;
//					cout << " [t,h] = [" << t << "," << h << "] > " << "invDay = " << t + invDay << endl ;
//					cout << " [t,h] = [" << t << "," << h << "] > " << "invHr = " << invHr << endl ;
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
		
		// OPTIMALITY GAP 2%
		//		cplex.setParam(IloCplex::EpGap, 0.02);
		cplex.extract(mod);
		cplex.exportModel("model.lp");
		
		cplex.solve();
		
		cplex.out() << "solution status = " << cplex.getStatus() << endl;
		cplex.out() << "objective value = " << cplex.getObjValue() << endl;
		
		cout << "# Vehicles = " << cplex.getValue(vTotal) << endl;
		
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
		cout << "y[1][h][i][1]-------------------" << endl ;
		for (h = 0; h < H; h++) {
			for (i = 0; i < M; i++) {
				if (cplex.getValue(y[1][h][i][0]) > 0)
					cout << "[h,i] = [" << h << "," << i + 1 << "] > " << cplex.getValue(y[1][h][i][0]) << endl ;																				
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
