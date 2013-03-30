/*
 *  definitions.h
 *  Vehicles_1st_echelon
 *
 *  Created by Guven Ince on 3/10/13.
 *  Copyright 2013 UMASS. All rights reserved.
 *
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

struct thSep : numpunct<char> {
	/* use space as separator */
	char do_thousands_sep() const { return ','; }
	
	/* digits are grouped by 3 digits each */
	string do_grouping() const { return "\3"; }
};

class Node {
private:
	int index;
	string name;
	int population;
public:
	//	Node(string name, int index){
	//		this->index = index;
	//		this->name = name;
	//	}
	int getPopulation(){
		return population;
	}
	void setIndex(int indx, string nam) {
		index = indx;
		name = nam;
	}
	int getIndex(){
		return index;
	}
	void setName(string nam){
		name = nam;
	}
	string getName(){
		return name;
	}
	
	void setPopulation(int pop) {
		population = pop;
	}
};

class DNode: public Node {
private:
	float demand;
	float mortality;
public:
	void setDemand(float dem){
		demand = dem;
	}
	
	void setMortality(float mor){
		mortality = mor;
	}
	
	float getDemand(){
		return demand;
	}
};

class SNode: public Node {
private:
	float supply;
public:
	void setSupply(float sup){
		supply = sup;
	}
	
	float getSupply(){
		return supply;
	}
};

class Edge {
private:
	float edgeVuln;
	int from;
	int to;
	Node frm;
	Node too;
	float edgeLength;
	//	int length;
public:
	Edge(int from, int to, float edgeVuln, float edgeLength){
		this->from = from;
		this->to = to;
		this->edgeVuln = edgeVuln;
		this->edgeLength = edgeLength;
	}
	
	Edge(Node from, Node to, float edgeVuln, float edgeLength){
		this->frm = from;
		this->too = to;
		this->edgeVuln = edgeVuln;
		this->edgeLength = edgeLength;
	}
	
	float getVulnerability() {
		return edgeVuln;
	}
	
	float getLength() {
		return edgeLength;
	}
};

class Cluster {
private:
	float demand;
	vector<DNode> nodes;
	string name;
public:
	void addNode(DNode n){
		nodes.push_back(n);
	}
	
	float getDemand(){
		float dem = 0;
		for(unsigned int k = 0; k < nodes.size(); k++)
			dem += nodes[k].getDemand();
		demand = dem;
		return demand;
	}
	
	void setName(string nm){
		name = nm;
	}
};

typedef IloArray<IloNumArray> NumMatrix;
typedef IloArray<IloNumArray2> Num3dMatrix;

typedef IloArray<IloIntArray> IntMatrix;

typedef IloArray<IloBoolVarArray> BoolVarMatrix;
typedef IloArray<BoolVarMatrix> BoolVar3dMatrix;

typedef IloArray<IloIntVarArray> IntVarMatrix;
typedef IloArray<IntVarMatrix>   IntVar3dMatrix;
typedef IloArray<IntVar3dMatrix> IntVar4dMatrix;
typedef IloArray<IntVar4dMatrix> IntVar5dMatrix;
typedef IloArray<IntVar5dMatrix> IntVar6dMatrix;

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix>   NumVar3dMatrix;
typedef IloArray<NumVar3dMatrix> NumVar4dMatrix;
typedef IloArray<NumVar4dMatrix> NumVar5dMatrix;
typedef IloArray<NumVar5dMatrix> NumVar6dMatrix;

extern IloInt M;
extern IloInt N;
extern IloInt T;
extern IloInt K;
extern IloInt B;
extern IloInt C;
extern IloInt H;
extern IloInt i, j, t, k, h, b;
extern IloInt a1, a2, a3;

extern vector<vector<int> > nStates;

extern IloEnv env;
extern IloModel mod;

extern vector<DNode> districts;
extern vector<SNode> suppliers;
extern vector<Cluster> clusters;

extern set<int> bundle;
extern vector<set<int> > bundles;

//PARAMETERS
extern IntMatrix tF;
extern IntMatrix tB;
extern IloNumArray P;
extern IloInt V;
extern IloNumArray w;
extern IloNumArray beta;
extern IloNum W;
extern IloIntArray Vin;
extern IloNumArray S; //not used here
extern IloNumArray teta; //not used here
extern IloNumArray tau; //not used here
extern IloNumArray pComm; //not used here
extern IloNumArray goal; //not used here
extern IloNumArray pGoal; //not used here
extern NumMatrix drtn; //not used here
extern NumMatrix D; //not used here

//VARIABLES
extern IloNumVar d1, d2; //not used here
extern NumVar3dMatrix s; //not used here
extern NumVarMatrix sT; //not used here
extern NumVar3dMatrix I; //not used here
extern IloNumVarArray Sk; //not used here

extern IntVar3dMatrix J; //not used here
extern IntVar3dMatrix E; // modeldeki \xi  //not used here
extern IntVar4dMatrix zeta; //modeldeki \zeta //not used here
extern IntVar6dMatrix X; //not used here
extern IntVar5dMatrix p; //not used here
extern IntVar3dMatrix m; //not used here

extern NumVar5dMatrix x;
extern IloIntVar tV;
extern IloIntVarArray vT;
extern IntVar3dMatrix v;
extern IntVar3dMatrix g;
extern IntVar3dMatrix rho;
extern IntVar4dMatrix y;
extern IntVar4dMatrix gama;
extern IntVar3dMatrix vSF;

#endif /* DISASTER_ALLOCATION_DEFS_H_ */
