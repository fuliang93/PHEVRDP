#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 28 23:35:50 2022

@author: wufuliang
"""


from para_solomon_sd import *

# =============================================================================
# Callback, subtour elimination, time flow, battery flow
# =============================================================================
def sepCut(model,where):
    
    if where == GRB.Callback.MIPSOL:
        Xvals = model.cbGetSolution(model._Xs)        
        routes = [(i,j) for (i,j) in Xvals.keys() if Xvals[i,j] > 0.5]
        sepCuts = cvrpsep.Sep(n-1,cDem[1:n],vCap,routes)
        if len(sepCuts) >= 0.9:
            for sepCut in sepCuts:
                if sum(Xvals[i,j] for (i,j) in Xvals.keys() if i in sepCut and j not in sepCut) <= math.ceil(sum(cDem[i] for i in sepCut)/vCap) - 10**(-6):
                    model.cbLazy(gp.quicksum(model._Xs[i,j] for (i,j) in arc if i in sepCut and j not in sepCut) >= math.ceil(sum([cDem[i] for i in sepCut])/vCap))
                    model._cbLaz += 1
                    
    elif where == GRB.Callback.MIPNODE:        
        status = model.cbGet(GRB.Callback.MIPNODE_STATUS) 
        if status == GRB.OPTIMAL:
            numCnt = model.cbGet(GRB.Callback.MIPNODE_NODCNT)
            if numCnt == 0 or model._cbCuts <= len(arc):                
                Xrel = model.cbGetNodeRel(model._Xs)           
                Xnet = {}
                for ele in Xrel.keys():
                    if Xrel[ele] > 0.5:
                        Xnet[ele] = Xrel[ele]
                        
                routes = [(i,j) for (i,j) in Xnet.keys()]  
                sepCuts = cvrpsep.Sep(n-1,cDem[1:n],vCap,routes)
                if len(sepCuts) >= 0.5:        
                    for sepCut in sepCuts:
                        if sum(Xnet[i,j] for (i,j) in Xnet.keys() if i in sepCut and j not in sepCut) < math.ceil(sum(cDem[i] for i in sepCut)/vCap) * 0.99: # Only add cuts when it is violated by 1%
                            model.cbCut(gp.quicksum(model._Xs[i,j] for (i,j) in arc if i in sepCut and j not in sepCut) >= math.ceil(sum([cDem[i] for i in sepCut])/vCap))                             
                            model._cbCuts += 1
                            


fils, Objs, Objgaps, Caltims, NumOfVehs, subErrs = [], [], [], [], [], [] # objective function and calculation time Dists                                                                                             
for filNcus in filNcuLs:

    file = filNcus[0]
    ncus = filNcus[1]                                                                                       

    nv = nvDic[file,ncus] + 1 # number of vehicles # Minimum + 1
    n = ncus + 1 # number of nodes, including the depot
    cDem, tMax, vCap, Cbat, arcDist, graDist, vUbDist, vMinDist, EMaxDist, EMinDist, twDist, tSer , arc, tMinDist, L, vL, EDist = dataRead(ncus,file,5/3.6) # vDis = 5 km/h
        
    print(filNcuLs.index(filNcus), file, ";", 'Number of vehicles', ':', nv, ";", 'Number of customers', ':', ncus, '\n')

    ##### Build the model #####
    m = gp.Model()
    
    # three index
    inds,indsP,indsN = [ind for ind in EDist.keys()], [ind for ind in EDist.keys() if EDist[ind] >= 0], [ind for ind in EDist.keys() if EDist[ind] < 0]  
    
    Xf = m.addVars(indsP, vtype=GRB.BINARY, name='xf') # x_{ij}^f
    Xe = m.addVars(indsP, vtype=GRB.BINARY, name='xe') # x_{ij}^e
    Xb = m.addVars(indsP, vtype=GRB.BINARY, name='xb') # x_{ij}^b
    Xr = m.addVars(indsN, vtype=GRB.BINARY, name='xr') # x_{ij}^r
    Xs = m.addVars(arc, vtype=GRB.BINARY, name='xs') # x_{ij}
    
    # Infeasible electric and boost modes
    m.addConstrs(Xe[i,j,k] == 0 for (i,j,k) in indsP if EMinDist[i,j] > Cbat)
    m.addConstrs(Xb[i,j,k] == 0 for (i,j,k) in indsN if EMinDist[i,j]*mu > Cbat)

    ls = m.addVars(range(0,n), lb = 0, ub = Cbat, name='l') # l_{i}; (2.31)
    ts = m.addVars(range(0,n+1), name='t') # t_i; 
        
    ### X Constraints ###    
    # Number of vehicles
    m.addConstr(gp.quicksum(Xs[ind] for ind in arc if ind[0] == 0) <= nv)
    m.addConstr(gp.quicksum(Xs[ind] for ind in arc if ind[0] == 0) >= math.ceil(sum(cDem)/vCap))
    
    # Customers
    m.addConstrs(gp.quicksum(Xs[ind] for ind in arc if ind[0] == i) == 1 for i in range(1,n))

    # Flow constraints
    m.addConstrs(gp.quicksum(Xs[ind] for ind in arc if ind[0] == i) == gp.quicksum(Xs[ind] for ind in arc if ind[1] == i) for i in range(1,n))  
    
    # Each arc
    m.addConstrs(gp.quicksum(Xf[ind] + Xe[ind] + Xb[ind] for ind in indsP if (ind[0],ind[1]) == (i,j)) + gp.quicksum(Xr[ind] for ind in indsN if (ind[0],ind[1]) == (i,j)) == Xs[i,j] for (i,j) in arc)        
    
    # battery flow
    m.addConstr(ls[0] == Cbat) # initial state of charge
    m.addConstrs(ls[i] - gp.quicksum((Xe[ind] + mu*Xb[ind])*EDist[ind] for ind in indsP if (ind[0],ind[1]) == (i,j)) - gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN if (ind[0],ind[1]) == (i,j)) >= ls[j] - (1 - Xs[i,j])*Cbat for (i,j) in arc if j != n) 
    m.addConstrs(ls[i] - gp.quicksum((Xe[ind] + mu*Xb[ind])*EDist[ind] for ind in indsP if (ind[0],ind[1]) == (i,j)) - gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN if (ind[0],ind[1]) == (i,j)) <= ls[j] + (1 - Xs[i,j])*Cbat for (i,j) in arc if j != n)
    m.addConstrs(ls[i] - gp.quicksum((Xe[ind] + mu*Xb[ind])*EDist[ind] for ind in indsP if (ind[0],ind[1]) == (i,j)) >= 0 for (i,j) in arc if j == n) 
    m.addConstrs(ls[i] - gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN if (ind[0],ind[1]) == (i,j)) <= Cbat for (i,j) in arc if j == n)
        
    # Time windows
    m.addConstrs(ts[i] >= twDist[i][0] for i in range(n)) #
    m.addConstrs(ts[i] <= twDist[i][1] for i in range(n)) #
    m.addConstrs(ts[i] + arcDist[i,j]*(gp.quicksum([(Xf[ind] + Xe[ind] + Xb[ind])/vL[ind] for ind in indsP if (ind[0],ind[1]) == (i,j)]) + gp.quicksum(Xr[ind]/vL[ind] for ind in indsN if (ind[0],ind[1]) == (i,j))) + tSer[i] - ts[j] <= (1 - Xs[i,j])*(twDist[i][1] + tSer[i] + arcDist[i,j]/vMin) for (i,j) in arc) # (2.32)    
    
    # =============================================================================
    # Objective function
    # =============================================================================
    obj = m.addVar(lb = - GRB.INFINITY, name='obj') #        
    m.addConstr(obj == gp.quicksum((cf*Xf[ind] + ce*Xe[ind] + cb*Xb[ind])*EDist[ind] for ind in indsP) + ce*gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN))
    m.setObjective(obj, GRB.MINIMIZE)
    
    # =============================================================================
    # Valid inequality 
    # =============================================================================
    lss = m.addVars(range(1,n), lb = 0, name='ls') # ls_i
    m.addConstrs(ls[i] - gp.quicksum((Xe[ind] + Xb[ind]*mu)*EDist[ind] for ind in indsP if (ind[0],ind[1]) == (i,n)) - gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN if (ind[0],ind[1]) == (i,n)) <= lss[i] + (1-Xs[i,n])*Cbat for i in range(1,n))
    m.addConstr(gp.quicksum((Xe[ind] + mu*Xb[ind])*EDist[ind] for ind in indsP) + gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN) <= Cbat*gp.quicksum(Xs[i,j] for (i,j) in arc if i == 0) - gp.quicksum(lss[ele] for ele in lss.keys()))
 
    
    # Lower bound inequalities
    m.addConstr( gp.quicksum((cf*Xf[ind] + ce*Xe[ind] + cb*Xb[ind])*EDist[ind] for ind in indsP) >= cf*gp.quicksum((Xf[ind] + Xe[ind] + Xb[ind])*EDist[ind] for ind in indsP)
                - (cf-cb)/mu*(Cbat*gp.quicksum(Xs[i,j] for (i,j) in arc if i == 0) - gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN) - gp.quicksum(lss[ele] for ele in lss.keys())) )

    m.addConstr( gp.quicksum((cf*Xf[ind] + ce*Xe[ind] + cb*Xb[ind])*EDist[ind] for ind in indsP) >= (cb-mu*ce)/(1-mu)*gp.quicksum((Xf[ind] + Xe[ind] + Xb[ind])*EDist[ind] for ind in indsP)
                - (cb-ce)/(1-mu)*(Cbat*gp.quicksum(Xs[i,j] for (i,j) in arc if i == 0) - gp.quicksum(Xr[ind]*EDist[ind] for ind in indsN) - gp.quicksum(lss[ele] for ele in lss.keys())) )
    
    # =============================================================================
    m._ts = ts
    m._Xs = Xs
    m._Xf,m._Xb,m._Xe,m._Xr = Xf,Xb,Xe,Xr
    m._ls = ls
            
    m._cbCuts, m._cbLaz = 0, 0
    m.Params.lazyConstraints = 1
    m.setParam("TimeLimit",timLimit) # Limit the solution time; Seconds
    m.Params.MIPGap = 10**(-4)
    
    # =============================================================================
    try:
        startTime = time.time()
        
        m.update()
        m.read("ini_result/" + program.split('_')[0] + "_io-" + filNcus[0].split('.')[0] + '_' + str(filNcus[1]) + "_ini_solution.sol")
        
        m.optimize(sepCut)
        endTime =  time.time()
        CalTime = endTime - startTime
        print ("--- %s seconds" % (endTime - startTime))
        print('Optimal cost: %g' % m.objVal)
        print('Number of vehicles', sum([Xs[i,j].x for (i,j) in arc if i == 0 and Xs[i,j].x >= 0.5]))
        print("Added "+str(m._cbCuts)+" cuts." + str(m._cbLaz) + " lazs")
        sucess = True
    except:
        print('Model is Inf')
        sucess = False
        
    if sucess == True:
        fils.append(file.split('.')[0] + '_' + str(ncus)) # File names
        Objs.append(m.objVal*eScale)  # Objective values
        Objgaps.append(m.mipgap) # Objective gaps
        Caltims.append(CalTime) # Calculation times
        
        ResDict = {'Objective': Objs, 'Caltime': Caltims, 'Objgaps': Objgaps}
        ResDf = pd.DataFrame(ResDict, index=fils)
        ResDf.to_csv('result/' + program.split('.')[0] + '.csv',index=True,sep=',',encoding='utf_8_sig')
        
    else:
        pass

