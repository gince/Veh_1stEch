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
		
//		const char* datafile  = "data.txt";
        const char* datafile  = "/Users/gince/research/xcodes/Veh_1stEch";
		ifstream dfile(datafile);
		if ( !dfile ) {
			cerr << "ERROR: could not open file '" << datafile
			<< "' for reading" << endl;
			throw (-1);
		}
		
		dfile >> P >> tF >> tB >> tmin >> tmax >> w >> W >> V;
		
		vector<DNode> districts = getDNodes(); // AVC, BAH, BAK, ..., SIL;
		vector<SNode> suppliers = getSNodes();
		vector<Cluster> clusters = getClusters();
		
		int tP = 0;
		for (j = 0; j < N; j++)
			tP += P[j];
		
		vT = IloIntVarArray(env, T, 0, tP);
		for (t=0; t < T; t++) {
			v[t] = IloIntVarArray(env, M, 0, tP);
			g[t] = IloIntVarArray(env, M, 0, tP);
			rho[t] = IloIntVarArray(env, N, 0, tP);
		}
		
		for (t=1; t < T; t++) {
			y[t] = IntVarMatrix(env, M);
			gama[t] = IntVarMatrix(env, N);
			z[t] = IloBoolVarArray(env, N);
			for(j=0; j < N; j++) {
				gama[t][j] = IloIntVarArray(env, M, 0, tP);
			}
			for(i=0; i < M; i++) {
				y[t][i] = IloIntVarArray(env, N, 0, tP);
			}
		}
		
		for (q = 0; q < Q; q++) {
			vSF[q] = IloIntVarArray(env, N, 0, tP);
		}
		
		
		//OBJECTIVE0 :: Minimize total shortfall in vehicle capacity
		IloExpr tvSF(env);
		for (q = 0; q < Q; q++)
			for (j = 0; j < N; j++)
				tvSF += vSF[q][j];
		
		IloExpr tvB(env);
		for (t = 1; t < T; t++)
			for (j = 0; j < N; j++)
				for (i = 0; i < M; i++)
					tvB += gama[t][j][i];
		
		IloExpr tvF(env);
		for (t = 1; t < T; t++)
			for (j = 0; j < N; j++)
				for (i = 0; i < M; i++)
					tvF += y[t][i][j];
		
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
		//		mod.add(tvB >= 117);
		//		mod.add(tvSF <= 118);
		//		mod.add(vT[0] == 0);
		/*		IloExpr tVQ1(env);
		 for (t = 0; t < 25; t++) {
		 tVQ1 += vT[t];
		 }
		 mod.add(tVQ1 == 50);
		 tVQ1.end();
		 IloExpr tVQ2(env);
		 for (t = 25; t < 49; t++) {
		 tVQ2 += vT[t];
		 }
		 mod.add(tVQ2 == 30);
		 tVQ2.end();
		 IloExpr tVQ3(env);
		 for (t = 49; t < 73; t++) {
		 tVQ3 += vT[t];
		 }
		 mod.add(tVQ3 == 20);
		 tVQ3.end();
		 IloExpr tVQ4(env);
		 for (t = 73; t < 97; t++) {
		 tVQ4 += vT[t];
		 }
		 mod.add(tVQ4 == 0);
		 tVQ4.end();
		 IloExpr tVQ5(env);
		 for (t = 97; t < 121; t++) {
		 tVQ5 += vT[t];
		 }
		 mod.add(tVQ5 == 0);
		 tVQ5.end();
		 */	
		/*		mod.add(v[0][0] == 25);
		 mod.add(v[0][1] == 23);
		 mod.add(v[0][2] == 25);
		 for (t = 1; t < 121; t++) {
		 for (i = 0; i < M; i++) {
		 mod.add(v[t][i] == 0);
		 }
		 }
		 */	
		mod.add(tvSF == 0);
		
		//VEHICLE CONSTRAINTS
		cout << "CONSTRAINT V-1" << endl;
		IloExpr vTT(env);
		for (t = 0; t < T; t++)	{
			vTT += vT[t];
		}
		mod.add(vTT <= tV);
		//		mod.add(vTT <= V);
		vTT.end();
		
		cout << "CONSTRAINT V-2" << endl;
		for (t = 0; t < T; t++) {
			IloExpr vITI(env);
			for (i = 0; i < M; i++)
				vITI += v[t][i];
			mod.add(vITI <= vT[t]);
			vITI.end();
		}
		
		cout << "CONSTRAINT V-3" << endl;
		for (i = 0; i < M; i++)
			mod.add(g[0][i] == v[0][i]);
		
		cout << "CONSTRAINT V-4" << endl;
		for (j = 0; j < N; j++)
			mod.add(rho[0][j] == 0);
		
		cout << "CONSTRAINT V-5" << endl;
		for (t = 1; t < T; t++) {
			for (i = 0; i < M; i++) {
				IloExpr yJ(env);
				IloExpr gamaJ(env);
				for (j = 0; j < N; j++) {
					yJ += y[t][i][j];
					if (t > tB[j][i]) {
						gamaJ += gama[t-tB[j][i]][j][i];
					}
				}
				mod.add(g[t-1][i] + v[t][i] + gamaJ - g[t][i] - yJ == 0);
				yJ.end();
				gamaJ.end();
			}
		}
		
		cout << "CONSTRAINT V-6" << endl;
		for (t = 1; t < T; t++)
			for (j = 0; j < N; j++) {
				IloExpr yI(env);
				IloExpr gamaI(env);
				for (i = 0; i < M; i++) {
					if (t > tF[i][j])
						yI += y[t-tF[i][j]][i][j];
					gamaI += gama[t][j][i];
				}
				mod.add(rho[t-1][j] + yI - rho[t][j] - gamaI == 0);
				yI.end();
				gamaI.end();
			}
		
		cout << "CONSTRAINT V-7" << endl;
		IloExpr tw(env);
		for(k = 1; k < K; k++)
			tw += w[k];
		for (q = 0; q < Q; q++)
			for (j = 0; j < N; j++){
				IloExpr yIT(env);
				for (t = tmin[j][q]; t <= tmax[j][q]; t++)
					for (i = 0; i < M; i++) {
						if (t > tF[i][j])
							yIT += y[t - tF[i][j]][i][j];
					}
				mod.add(yIT + vSF[q][j] - ceil((17/W)*P[j]) == 0);
				//				mod.add(yI - (tw/W)*P[j] >= 0);
				yIT.end();
			}
		tw.end();
		
		/*		cout << "CONSTRAINT V-8" << endl;
		 for (q = 0; q < Q; q++) {
		 for (j = 0; j < N; j++) {				
		 IloExpr tz(env);
		 for (t = tmin[j][q]; t <= tmax[j][q]; t++) {
		 tz += z[t][j];
		 }
		 mod.add(tz == 1);
		 tz.end();
		 }
		 }
		 
		 cout << "CONSTRAINT V-9";
		 for (t = 1; t < T; t++)
		 for (j = 0; j < N; j++) {
		 IloExpr yI(env);
		 for (i = 0; i < M; i++) {
		 if (t > tF[i][j])
		 yI += y[t - tF[i][j]][i][j];
		 }
		 mod.add(z[t][j] <= yI <= V*z[t][j]);
		 yI.end();
		 }
		 */		
		cout << "CONSTRAINT SON" << endl;
		
		IloCplex cplex(env);
		cplex.extract(mod);
		cplex.exportModel("model.lp");
		
		
		cplex.solve();
		
		cplex.out() << "solution status = " << cplex.getStatus() << endl;
		cplex.out() << "objective value = " << cplex.getObjValue() << endl;
		/*		
		 cout << "TEST SON" << endl;
		 for (j = 0; j < N; j++) {
		 for (t = 1; t < T; t++) {
		 if (cplex.getValue(z[t][j]) > 0) {
		 cout << "t = " << t << ", j = " << j << " : " << cplex.getValue(z[t][j]) << endl;
		 }
		 }
		 }
		 */
		IloExpr vTotal(env);
		for (t = 0; t < T; t++) {
			for (i = 0; i < M; i++)
				vTotal += v[t][i];
		}
		cout << "# Vehicles = " << cplex.getValue(vTotal) << endl;
		cout << "Total vehicle shortfall = " << cplex.getObjValue() << endl;
		cout << "u[t][j]-------------------" << endl;
		for(q = 0; q < Q; q++){
			cout << "DAY " << q << " __________________" << endl;
			for(j = 0; j < N; j++){
				if (cplex.getValue(vSF[q][j]) > 0) {
					cout << "j = " << j + 1 << " > " << cplex.getValue(vSF[q][j]) << endl;
				}
			}
		}
		
		cout << "vT[t]-------------------" << endl;
		for(t = 0; t < T; t++)
			if (cplex.getValue(vT[t]) > 0){
				cout << "t = " << t << "\t > ";
				cout << cplex.getValue(vT[t]) << endl;
			}
		
		cout << "v[t][i]-------------------" << endl;
		for(t = 0; t < T; t++){
			for(i = 0; i < M; i++){
				if (cplex.getValue(v[t][i]) > 0) {
					cout << "t = " << t << ", i = " << i + 1 << " > " << cplex.getValue(v[t][i]) << "\t" << endl;
				}
			}
		}
		
		cout << "vehicle flow around supplier sites" << endl;
		cout << "departures from suppliers" << endl;
		for (t = 1; t < 121; t++) {
			if ((t - 1) % 24 == 0) {
				cout << "DAY " << ceil(t/24) + 1 << "-----------" << endl;
			}
			for (i = 0; i < M; i++) {
				//				cout << "i = " << i + 1 << endl;
				for (j = 0; j < N; j++) {
					if (cplex.getValue(y[t][i][j]) > 0) {
						cout << cplex.getValue(y[t][i][j]) << " at t = " << t << " from i = " << i + 1 << " to j = " << j + 1 << endl;
					}
				}
			}
		}
		
		cout << "arrivals at suppliers" << endl;
		for (t = 1; t < 121; t++) {
			if ((t - 1) % 24 == 0) {
				cout << "DAY " << ceil(t/24) + 1 << "-----------" << endl;
			}
			for (i = 0; i < M; i++) {
				//				cout << "i = " << i + 1 << endl;
				for (j = 0; j < N; j++) {
					if (cplex.getValue(gama[t][j][i]) > 0) {
						cout << cplex.getValue(gama[t][j][i]) << " at t = " << t + tB[j][i]<< " to i = " << i + 1 << " from j = " << j + 1 << endl;
					}
				}
			}
		}
		
		cout << "vehicle flow around demand sites" << endl;
		cout << "departures from demand sites" << endl;
		for (t = 1; t < 121; t++) {
			if ((t - 1) % 24 == 0) {
				cout << "DAY " << ceil(t/24) + 1 << "-----------" << endl;
			}
			for (j = 0; j < N; j++) {
				for (i = 0; i < M; i++) {
					//				cout << "i = " << i + 1 << endl;
					if (cplex.getValue(gama[t][j][i]) > 0) {
						cout << cplex.getValue(gama[t][j][i]) << " at t = " << t << " from j = " << j + 1 << " to i = " << i + 1 << endl;
					}
				}
			}
		}
		
		cout << "arrivals at demand sites" << endl;
		for (t = 1; t < 121; t++) {
			if ((t - 1) % 24 == 0) {
				cout << "DAY " << ceil(t/24) + 1 << "-----------" << endl;
			}
			for (j = 0; j < N; j++) {
				for (i = 0; i < M; i++) {
					//				cout << "i = " << i + 1 << endl;
					if (cplex.getValue(y[t][i][j]) > 0) {
						cout << cplex.getValue(y[t][i][j]) << " at t = " << t + tF[i][j]<< " to j = " << j + 1 << " from i = " << i + 1 << endl;
					}
				}
			}
		}
		
		//		cout << "to the next day " << cplex.getValue(g[24][i]) << endl;
		cout << "_________________________" << endl;
		cout << "vehicles flow" << endl;
		cout << "DAY 1 __________________" << endl;
		cout << "i to j__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 1; t < 25; t++)
					if (cplex.getValue(y[t][i][j]) > 0)
						cout << "t = " << t << ", i = " << i + 1 << ", j = " << j + 1 << " > " << cplex.getValue(y[t][i][j]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "j to i__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 1; t < 25; t++)
					if (cplex.getValue(gama[t][j][i]) > 0)
						cout << "t = " << t << ", j = " << j + 1 << ", i = " << i + 1 << " > " << cplex.getValue(gama[t][j][i]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "DAY 2 __________________" << endl;
		cout << "i to j__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 25; t < 49; t++)
					if (cplex.getValue(y[t][i][j]) > 0)
						cout << "t = " << t << ", i = " << i + 1 << ", j = " << j + 1 << " > " << cplex.getValue(y[t][i][j]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "j to i__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 25; t < 49; t++)
					if (cplex.getValue(gama[t][j][i]) > 0)
						cout << "t = " << t << ", j = " << j + 1 << ", i = " << i + 1 << " > " << cplex.getValue(gama[t][j][i]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "DAY 3 __________________" << endl;
		cout << "i to j__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 49; t < 73; t++)
					if (cplex.getValue(y[t][i][j]) > 0)
						cout << "t = " << t << ", i = " << i + 1 << ", j = " << j + 1 << " > " << cplex.getValue(y[t][i][j]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "j to i__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 49; t < 73; t++)
					if (cplex.getValue(gama[t][j][i]) > 0)
						cout << "t = " << t << ", j = " << j + 1 << ", i = " << i + 1 << " > " << cplex.getValue(gama[t][j][i]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "DAY 4 __________________" << endl;
		cout << "i to j__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 73; t < 97; t++)
					if (cplex.getValue(y[t][i][j]) > 0)
						cout << "t = " << t << ", i = " << i + 1 << ", j = " << j + 1 << " > " << cplex.getValue(y[t][i][j]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "j to i__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 73; t < 97; t++)
					if (cplex.getValue(gama[t][j][i]) > 0)
						cout << "t = " << t << ", j = " << j + 1 << ", i = " << i + 1 << " > " << cplex.getValue(gama[t][j][i]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "DAY 5 __________________" << endl;
		cout << "i to j__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 97; t < 121; t++)
					if (cplex.getValue(y[t][i][j]) > 0)
						cout << "t = " << t << ", i = " << i + 1 << ", j = " << j + 1 << " > " << cplex.getValue(y[t][i][j]) << endl;
			cout << "_________________________" << endl;
		}
		cout << "j to i__________________" << endl;
		for (j = 0; j < N; j++) {
			for (i = 0; i < M; i++)
				for (t = 97; t < 121; t++)
					if (cplex.getValue(gama[t][j][i]) > 0)
						cout << "t = " << t << ", j = " << j + 1 << ", i = " << i + 1 << " > " << cplex.getValue(gama[t][j][i]) << endl;
			cout << "_________________________" << endl;
		}
		/*		
		 cout << "vehicles from j to i over time" << endl;
		 cout << "DAY 1 __________________" << endl;
		 for (j = 0; j < N; j++) {
		 for (i = 0; i < M; i++)
		 for (t = 1; t < 25; t++)
		 if (cplex.getValue(gama[t][j][i]) > 0)
		 cout << "t = " << t << ", j = " << j << ", i = " << i << " > " << cplex.getValue(gama[t][j][i]) << endl;
		 cout << "_________________________" << endl;
		 }
		 cout << "DAY 2 __________________" << endl;
		 for (j = 0; j < N; j++) {
		 for (i = 0; i < M; i++)
		 for (t = 25; t < 49; t++)
		 if (cplex.getValue(gama[t][j][i]) > 0)
		 cout << "t = " << t << ", j = " << j << ", i = " << i << " > " << cplex.getValue(gama[t][j][i]) << endl;
		 cout << "_________________________" << endl;
		 }
		 cout << "DAY 3 __________________" << endl;
		 for (j = 0; j < N; j++) {
		 for (i = 0; i < M; i++)
		 for (t = 49; t < 73; t++)
		 if (cplex.getValue(gama[t][j][i]) > 0)
		 cout << "t = " << t << ", j = " << j << ", i = " << i << " > " << cplex.getValue(gama[t][j][i]) << endl;
		 cout << "_________________________" << endl;
		 }
		 cout << "DAY 4 __________________" << endl;
		 for (j = 0; j < N; j++) {
		 for (i = 0; i < M; i++)
		 for (t = 73; t < 97; t++)
		 if (cplex.getValue(gama[t][j][i]) > 0)
		 cout << "t = " << t << ", j = " << j << ", i = " << i << " > " << cplex.getValue(gama[t][j][i]) << endl;
		 cout << "_________________________" << endl;
		 }
		 cout << "DAY 5 __________________" << endl;
		 for (j = 0; j < N; j++) {
		 for (i = 0; i < M; i++)
		 for (t = 97; t < 121; t++)
		 if (cplex.getValue(gama[t][j][i]) > 0)
		 cout << "t = " << t << ", j = " << j << ", i = " << i << " > " << cplex.getValue(gama[t][j][i]) << endl;
		 cout << "_________________________" << endl;
		 }
		 */		
		/*		for(j=0;j<N;j++) {
		 //			toLatexP(cplex, districts);
		 for(i=0;i<M;i++) {
		 //				toLatexXY(cplex, districts);
		 }
		 }
		 */
		
		/*
		 cout << "vSF[t][j]-------------------" << endl;
		 for(t = 1; t < T; t++){
		 cout << "t = " << t << endl;
		 for(j = 0; j < N; j++){
		 cout <<  cplex.getValue(vSF[t][j]) << "\t";
		 }
		 cout << endl;
		 }
		 */
		/*		
		 cout << "vT[t]-------------------" << endl;
		 for(t = 0; t < T; t++){
		 cout << "t = " << t << "\t";
		 cout << cplex.getValue(vT[t]) << endl;
		 }
		 
		 cout << "v[t][i]-------------------" << endl;
		 for(t = 0; t < T; t++){
		 cout << "t = " << t << endl;
		 for(i = 0; i < M; i++){
		 cout <<  cplex.getValue(v[t][i]) << "\t";
		 }
		 cout << endl;
		 }
		 
		 cout << "g[t][i]-------------------" << endl;
		 for(t = 1; t < T; t++){
		 cout << "t = " << t << endl;
		 for(i = 0; i < M; i++){
		 cout <<  cplex.getValue(g[t][i]) << "\t";
		 }
		 cout << endl;
		 }
		 
		 cout << "rho[t][j]-------------------" << endl;
		 for(t = 1; t < T; t++){
		 cout << "t = " << t << endl;
		 for(j = 0; j < N; j++){
		 cout <<  cplex.getValue(rho[t][j]) << "\t";
		 }
		 cout << endl;
		 }
		 
		 cout << "y[t][i][j]-------------------" << endl;
		 for(t = 1; t < T; t++){
		 cout << "t = " << t << endl;
		 for(i = 0; i < M; i++){
		 for(j = 0; j < N; j++)
		 cout <<  cplex.getValue(y[t][i][j]) << "\t";
		 cout << endl;
		 }
		 }
		 
		 cout << "gama[t][j][i]-------------------" << endl;
		 for(t = 1; t < T; t++){
		 cout << "t = " << t << endl;
		 for(j = 0; j < N; j++){
		 for(i = 0; i < M; i++)
		 cout <<  cplex.getValue(gama[t][j][i]) << "\t";
		 cout << endl;
		 }
		 }
		 
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
