/*
 *  functions.cpp
 *  Vehicles_1st_echelon
 *
 *  Created by Guven Ince on 3/10/13.
 *  Copyright 2013 UMASS. All rights reserved.
 *
 */


#include <ilcplex/ilocplex.h>
#include <set>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

#include "functions.h"

IloInt M = 3;
IloInt N = 5;
IloInt T = 6;
IloInt K = 3;
IloInt B = 8;
IloInt H = 3;
IloInt i, j, t, k, h, b;
IloInt a1, a2, a3;

vector<vector<int> > nStates(N, vector<int>(T));

IloEnv env;
IloModel mod(env);

DNode AVC, BAH, BAK, GUN, ZEY, BYG, EMI, FAT, BAG, BAY, ESE, EYU, GAZ, KUC, ADA, KAD, KAR, MAL, PEN, TUZ, BES, KAG, SAR, SIS, BEY, UMR, USK, BUY, CAT, SIL;
SNode  ANK, IST, IZM, MER, SAM;
Cluster EuMar, Historic, EuIn, AsMar, EuBos, AsBos, OutIMM;

set<int> bundle;
vector<set<int> > bundles(B);

//PARAMETERS
IloNumArray S(env);
IloNumArray P(env);
IloInt V;
IloNumArray w(env);
IloNumArray beta(env);
IloNum W;
IloIntArray Vin(env);
NumMatrix drtn(env);
IntMatrix tF(env);
IntMatrix tB(env);

//not used here
IloNumArray teta(env);
IloNumArray tau(env);
IloNumArray pComm(env);
IloNumArray goal(env);
IloNumArray pGoal(env);
NumMatrix D(env);


//VARIABLES
NumVar3dMatrix s(env, T);
NumVarMatrix sT(env, T);
NumVar3dMatrix I(env, T);
NumVar5dMatrix x(env, T);
IloNumVarArray Sk(env, K);

IntVar3dMatrix J(env, T);
IntVar3dMatrix E(env, T);
IntVar4dMatrix zeta(env, T);
IntVar6dMatrix X(env, T);
IntVar5dMatrix p(env, T);
IntVar3dMatrix m(env, T);
IloNumVar d1(env, 0, +IloInfinity);
IloNumVar d2(env, 0, +IloInfinity);

IloIntVar tV(env);
IloIntVarArray vT(env, T);
IntVar3dMatrix v(env, T);
IntVar3dMatrix g(env, T);
IntVar3dMatrix rho(env, T);
IntVar4dMatrix y(env, T);
IntVar4dMatrix gama(env, T);
IntVar3dMatrix vSF(env, T);

vector<SNode> getSNodes() {
	vector<SNode> suppliers;
	suppliers.push_back(ANK);
	suppliers.push_back(IST);
	suppliers.push_back(IZM);
	suppliers.push_back(MER);
	suppliers.push_back(SAM);
	suppliers[0].setName("ANK");
	suppliers[1].setName("IST");
	suppliers[2].setName("IZM");
	suppliers[3].setName("MER");
	suppliers[4].setName("SAM");
	return suppliers;
}

vector<DNode> getDNodes() {
	vector<DNode> dstrcts;
	dstrcts.push_back(AVC); dstrcts[0].setName("AVC");
	dstrcts.push_back(BAH); dstrcts[1].setName("BAH");
	dstrcts.push_back(BAK); dstrcts[2].setName("BAK");
	dstrcts.push_back(GUN); dstrcts[3].setName("GUN");
	dstrcts.push_back(ZEY); dstrcts[4].setName("ZEY");
	dstrcts.push_back(BYG); dstrcts[5].setName("BYG");
	dstrcts.push_back(EMI); dstrcts[6].setName("EMI");
	dstrcts.push_back(FAT); dstrcts[7].setName("FAT");
	dstrcts.push_back(BAG); dstrcts[8].setName("BAG");
	dstrcts.push_back(BAY); dstrcts[9].setName("BAY");
	dstrcts.push_back(ESE); dstrcts[10].setName("ESE");
	dstrcts.push_back(EYU); dstrcts[11].setName("EYU");
	dstrcts.push_back(GAZ); dstrcts[12].setName("GAZ");
	dstrcts.push_back(KUC); dstrcts[13].setName("KUC");
	dstrcts.push_back(ADA); dstrcts[14].setName("ADA");
	dstrcts.push_back(KAD); dstrcts[15].setName("KAD");
	dstrcts.push_back(KAR); dstrcts[16].setName("KAR");
	dstrcts.push_back(MAL); dstrcts[17].setName("MAL");
	dstrcts.push_back(PEN); dstrcts[18].setName("PEN");
	dstrcts.push_back(TUZ); dstrcts[19].setName("TUZ");
	dstrcts.push_back(BES); dstrcts[20].setName("BES");
	dstrcts.push_back(KAG); dstrcts[21].setName("KAG");
	dstrcts.push_back(SAR); dstrcts[22].setName("SAR");
	dstrcts.push_back(SIS); dstrcts[23].setName("SIS");
	dstrcts.push_back(BEY); dstrcts[24].setName("BEY");
	dstrcts.push_back(UMR); dstrcts[25].setName("UMR");
	dstrcts.push_back(USK); dstrcts[26].setName("USK");
	dstrcts.push_back(BUY); dstrcts[27].setName("BUY");
	dstrcts.push_back(CAT); dstrcts[28].setName("CAT");
	dstrcts.push_back(SIL); dstrcts[29].setName("SIL");
	/*
	 for(j=0; j<N; j++){
	 dstrcts[j].setDemand(P[j]);
	 tDemand += P[j];
	 }
	 tRefugees = ceil(4*tDemand/70);
	 */
	return dstrcts;
}

vector<Cluster> getClusters() {
	vector<Cluster> clusters;
	EuMar.setName("European Marmara");
	Historic.setName("Historic District");
	EuIn.setName("European Inland");
	AsMar.setName("Asean Marmara");
	EuBos.setName("European Bosphorus");
	AsBos.setName("Asean Bosphorus");
	OutIMM.setName("Outside IMM");
	clusters.push_back(EuMar);
	clusters.push_back(Historic);
	clusters.push_back(EuIn);
	clusters.push_back(AsMar);
	clusters.push_back(EuBos);
	clusters.push_back(AsBos);
	clusters.push_back(OutIMM);
	return clusters;
}

void toLatexP(IloCplex cplex, vector<DNode> dstrcts) {
	ofstream outfile;
	outfile.imbue(locale(std::locale(), new thSep));
	double scalefactor = .60;
	//	string fileName = "../../Tez/latex/dissertation/tables/toLatexP";
	string fileName = "/Users/guvenince/Desktop/dissertation/dissertation-jan6/tables/toLatexP";
	std::ostringstream oss;
	oss << fileName << j << ".table";
	outfile.open(oss.str().c_str());
	outfile.setf(ios::fixed, ios::floatfield);
	outfile << setprecision(0);
	outfile << "\\begin{table}[H]\\centering" << endl;
	outfile << "\\caption{Computational results of Example 2.X, where for $\\theta_k = 2,3,4, \\tau_k = 3,4,5$, $B_1 = \\{1\\}$, $B_2 = \\{2\\}$, $B_3 = \\{3\\}$, $B_4 = \\{1,2\\}$, $B_5 = \\{1,3\\}$, $B_6 = \\{2,3\\}$, $B_7 = \\{1,2,3\\}$}" << endl;
	int counter = 0;
	k = 0;
	t = 1;
	for(t = 1; t < T; t++){
		if (counter == 0){
			outfile << "\\scalebox{" << setprecision(1) << scalefactor << setprecision(0)<< "}{" << endl;
			outfile << "\t\\begin{tabular}{rCC|CCC|CCCCCCC}	" << endl;
			outfile << "&&&\\multicolumn{3}{c|}{$\\mu_{j,t}$}& \\multicolumn{7}{c}{$\\chi^r_{j,B,t}$} \\\\\\cline{4-13}" << endl;
			outfile << "$t$ & $R$ & $d^r_{j,t}$ & $k = 1$  & $k = 2$  & $k = 3$ & $B_1$ & $B_2$ & $B_3$ & $B_4$ & $B_5$ & $B_6$ & $B_7$ \\\\\\hline";
		}
		outfile << endl <<t;
		for (a1 = 0; a1 <= teta[0]+tau[0]; a1++)
			for (a2 = 1; a2 <= teta[1]+tau[1]; a2++)
				for (a3 = 1; a3 <= teta[2]+tau[2]; a3++){
					if(cplex.getValue(p[t][j][a1][a2][a3]) > 0){
						outfile << "& [" << a1<<a2<<a3<<"] & " << cplex.getValue(p[t][j][a1][a2][a3]);
						outfile << "& ";
						if(a1 == teta[0]+tau[0])
							outfile << cplex.getValue(p[t][j][a1][a2][a3]) << "& ";
						else outfile << 0 << "& ";
						if(a2 == teta[1]+tau[1])
							outfile << cplex.getValue(p[t][j][a1][a2][a3]) << "& ";
						else outfile << 0 << "& ";
						if(a3 == teta[2]+tau[2])
							outfile << cplex.getValue(p[t][j][a1][a2][a3]) << "& ";
						else outfile << 0;
						for(b = 1; b < B; b++)
							if((a1 < teta[0]+tau[0])&&(a2 < teta[1]+tau[1])&&(a3 < teta[2]+tau[2]))
								outfile << " & " << cplex.getValue(X[t][j][a1][a2][a3][b]);
							else outfile << " & " << 0;
						outfile << "\\\\" << endl;
						counter += 1;
						if(counter == 70){
							outfile << "\t\\end{tabular}" << endl;
							outfile << "}" << endl;
							outfile << "\\scalebox{" << setprecision(1) << scalefactor << setprecision(0)<< "}{" << endl;
							outfile << "\t\\begin{tabular}{rCC|CCC|CCCCCCC}	" << endl;
							outfile << "&&&\\multicolumn{3}{c|}{$\\mu_{k,t}$}& \\multicolumn{7}{c}{$\\chi^R_{t,B}$} \\\\\\cline{4-13}" << endl;
							outfile << "$t$ & $R$ & $\\eta^R_t$ & $k = 1$  & $k = 2$  & $k = 3$ & $B_1$ & $B_2$ & $B_3$ & $B_4$ & $B_5$ & $B_6$ & $B_7$ \\\\\\hline";
							counter = 0;
							outfile << t;
						}
					}
				}
		outfile << "\\hline";
	}
	outfile << "\t\\end{tabular}" << endl;
	outfile << "}" << endl;
	outfile << endl << "\\label{tab:XXXXXXX}" << endl;
	outfile << "\\end{table}" << endl;
	//	if (t != T-1)
	//		outfile << "\\vspace{-6mm}" << endl;
	//	}
}

void toLatexXY(IloCplex cplex, vector<DNode> dstrcts) {
	ofstream outfile;
	outfile.imbue(locale(std::locale(), new thSep));
	//	double scalefactor = .60;
	//	string fileName = "../../Tez/latex/dissertation/tables/toLatexP";
	string fileName = "/Users/guvenince/Desktop/dissertation/dissertation-jan6/tables/toLatexXY";
	std::ostringstream oss;
	oss << fileName << i << j << ".table";
	outfile.open(oss.str().c_str());
	outfile.setf(ios::fixed, ios::floatfield);
	outfile << setprecision(0);
	//	outfile << "\\begin{table}[H]\\centering" << endl;
	//	outfile << "\\caption{Supplier Site Results :: i = " << i << " and j = " << j << " }" << endl;
	int counter = 0;
	k = 0;
	t = 1;
	for(t = 1; t < T; t++){
		if (counter == 0){
			//			outfile << "\\scalebox{" << setprecision(1) << scalefactor << setprecision(0)<< "}{" << endl;
			outfile << "\t\\begin{tabular}{r|ccc|cccc}	" << endl;
			outfile << "&\\multicolumn{3}{c|}{$x_{i,j,k,t}$} & & \\\\\\cline{2-8}" << endl;
			outfile << i << "$\\rightarrow$" << j << " & $k = 1$  & $k = 2$  & $k = 3$ & $y_{i,j,t}$  & $\\gamma_{j,i,t}$ & $g_{i,t}$ & $v_{i,t}$ \\\\\\hline";
		}
		outfile << endl <<t;
		for(k=0; k < K; k++)
//			outfile << "& " << cplex.getValue(x[t][i][j][k]);
//		outfile << "& " << cplex.getValue(y[t][i][j]);
//		outfile << "& " << cplex.getValue(gama[t][j][i]);
//		outfile << "& " << cplex.getValue(g[t][i]);
//		outfile << "& " << cplex.getValue(v[t][i]);
		counter += 1;
		outfile << "\\\\\\hline";
	}
	outfile << endl << "\t\\end{tabular}" << endl;
	//	outfile << "}" << endl;
	//	outfile << endl << "\\label{tab:XXXXXXX}" << endl;
	//	outfile << "\\end{table}" << endl;
}

