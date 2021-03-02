#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug  7 12:46:04 2020

@author: binyao
"""

from gurobipy import *
import csv
import numpy as np
from math import *
from itertools import product
import time as tm
import random

# read index of cluster
cluster = [] 
center = 15772 #NEED MODIFY!!!!!!
with open('15772.txt',"r") as f:  #TXT FILE!!!!!!
    read_data=f.read()
    a=read_data.split()
    for i in range(len(a)):
        cluster.append(int(float(a[i])))

#read demand also decide which days to read
demand_data = []
count = 0
with open('mopta2020_q2018.csv') as f:
    f_csv = csv.reader(f)
    for row in f_csv:
        demand_data.append([int(row[i]) for i in range(len(row))])
'''
#find the index of day with greatest demand
max_dem = -1
max_idx = -1
for i in range(1,366):
    day_tot_dem = sum(demand_data[j][i] for j in range(len(demand_data)))
    if day_tot_dem > max_dem:
        max_dem = day_tot_dem
        max_idx = i
'''
days = [] # index of the days NEED INPUT!!!!!!
season = [1,91,182,274,366]
for i in range(4):
    season_tot_dem = 0
    day_tot_dem = []
    for j in range(season[i],season[i+1]):
        season_tot_dem += sum(demand_data[k][j] for k in range(len(demand_data)))
        day_tot_dem.append(sum(demand_data[k][j] for k in range(len(demand_data))))
    avg_sea_dem = season_tot_dem/(season[i+1]-season[i])  
    d_y_t = [abs(day_tot_dem[j]-avg_sea_dem) for j in range(len(day_tot_dem))]
    opt_idx = d_y_t.index(min(d_y_t))
    days.append(season[i]+opt_idx)
#days.insert(0,max_idx)


#ct = 0
tim = []
veh = []
cos = []
v_cost = 30
for day in days:
    '''
    if ct == 0:
        v_cost = 300
    else:
        v_cost = 30
    '''
    #create daily demand and index set
    demand = []
    index = []
    for i in cluster:
        for j in range(len(demand_data)):
            if demand_data[j][0] == center:
                center_index = j
            elif i == demand_data[j][0] and demand_data[j][day]>0:
                demand.append(demand_data[j][day])
                index.append(j)
    index.append(center_index)
    #create distance set
    dist = [[-1 for i in range(len(index))] for j in range(len(index))]
    distance = []
    with open('V1.csv') as f:
        f_csv = csv.reader(f)
        for row in f_csv:
            r = [float(i) for i in row]
            distance.append(r)
    
    for i in range(len(index)):
        for j in range(len(index)):
            dist[i][j] = distance[index[i]][index[j]]
    
    m = len(demand) #number of customer
    print('m',m,'\n')
    if m==0:
        continue
    if m < 12: 
        #master LP
        obj_ite = []
        time_ite = []
        start_time = tm.time()
        master = Model("VRP")
        master.setParam("OutputFlag",0)
        
        #Initialization: generate initial routes using pseudo-sweep heuristic
        A = [] #set of columns or matrix of master problem
        c = [] #set of cost of each column in A
        s = [0]*m #s stores the route generated
        s[0] = 1
        count = 1
        cap = demand[0] 
        time = 1/40*dist[m][0]
        st_time = 1/40*dist[m][0]
        while count<=m-1:
            time += 1/40*dist[count-1][count]
            cap += demand[count]
            ed_time = 1/40*dist[count][m]
            tot_time = time+ed_time
            if cap<=60 and tot_time<=11 and tot_time-st_time<=10 and time-st_time<=8:
                s[count] = 1
            else:
                A.append(s)
                start = s.index(1)
                end = start+sum(s)-1
                c.append(0.7*(dist[m][start]+sum(dist[i][i+1] for i in range(start,end))+dist[end][m]))
                s = [0]*m
                s[count] = 1
                cap = demand[count]
                time = 1/40*dist[m][count]
                st_time = 1/40*dist[m][count]
            count += 1
        A.append(s)
        start = s.index(1)
        end = start+sum(s)-1
        c.append(0.7*(dist[m][start]+sum(dist[i][i+1] for i in range(start,end))+dist[end][m]))
        
        node = {}
        z = {}
        n = len(A)
        for j in range(n):
            z[j] = master.addVar(obj=c[j]+v_cost, vtype=GRB.CONTINUOUS, name="num[%d]"%j) #c[j]+30
        for i in range(m):
            node[i] = master.addConstr(quicksum(A[j][i]*z[j] for j in range(n)) >= 1, name="dem[%d]"%i)
        master.update()  
        
        #adjust the graph structure
        d = dist[-1]
        for i in range(2):
            dist.append(d+[0,0])
        for i in range(len(dist)-2):
            dist[i] += [dist[i][-1]]*2
        demand += [0,0,0]
        
        nodes = range(m)
        E = list(product(nodes, nodes))
        for j in nodes:
            E.remove((j,j))
        for i in nodes:
            for j in range(m+1,m+3):
                E.append((i,j))
        for j in nodes:
            for i in range(m,m+2):
                E.append((i,j))
        E1 = [E[i] for i in range(len(E))]
        for j in nodes:
            E1.remove((m,j))
        E2 = [E1[i] for i in range(len(E1))]
        for i in nodes:
            E2.remove((i,m+2))
        E3 = [E2[i] for i in range(len(E2))]
        for i in nodes:
            E3.remove((i,m+1))
            E3.remove((m+1,i))
        
        while 1:
        #resolve master LP
            master.optimize()
            obj_ite.append(master.ObjVal)
            print("t",obj_ite[-1],'\n')
            end_time = tm.time()
            time_ite.append(end_time-start_time)
            profit = [node[i].Pi for i in range(m)]    #optimal dual vars 
            #print(n,profit,'')
        #pricing problem
            #start_time = tm.time()
            pricing = Model("search route")
            pricing.reset(0)
            pricing.setParam("OutputFlag",0)
            customer = pricing.addVars(nodes, vtype=GRB.BINARY, name="customer")
            arc = pricing.addVars(E, vtype=GRB.BINARY, name='arc')
            l = pricing.addVars(nodes, ub=60, vtype=GRB.CONTINUOUS, name="left loading")
            pricing.setObjective(0.7*quicksum(dist[i][j]*arc[i,j] for (i,j) in E)+v_cost-quicksum(profit[i]*customer[i] for i in range(m)), GRB.MINIMIZE)
            pricing.addConstr(quicksum(arc[m,j] for j in nodes) == 1, name='s->')
            pricing.addConstr(quicksum(arc[i,m+2] for i in nodes) == 1, name='->t')
            pricing.addConstr(quicksum(arc[m+1,j] for j in nodes) - quicksum(arc[i,m+1] for i in nodes) == 0, name='->o->')
            for i in nodes: 
                N = [j for j in range(m+3) if j not in [i,m]]
                pricing.addConstr(quicksum(arc[i,j] for j in N) == customer[i], name='c->')
                P = [j for j in range(m+2) if j != i]
                pricing.addConstr(quicksum(arc[j,i] for j in P) == customer[i], name='->c')
                pricing.addConstr(l[i]+demand[i]-60 <= (1-arc[m,i])*30, name='load'+'s'+str(i))
                pricing.addConstr(l[i]+demand[i]-60 <= (1-arc[m+1,i])*30, name='load'+'o'+str(i))
            for (i,j) in E3:
                pricing.addConstr(l[j]+demand[j]-l[i] <= (1-arc[i,j])*30, name='load'+str(i)+str(j))
            pricing.addConstr(quicksum(dist[i][j]*arc[i,j] for (i,j) in E) <= 440, name='time')
            pricing.addConstr(quicksum(dist[i][j]*arc[i,j] for (i,j) in E1) <= 400, name='time_1')
            pricing.addConstr(quicksum(dist[i][j]*arc[i,j] for (i,j) in E2) <= 320, name='time_2')
            
            flag = False        #keep track of whether a single cycle is found
            while(flag == False):         #iterations of the constraint-generation algorithm
                pricing.optimize()
                #end_time = tm.time() 
                #print(n, "Time = "+str(end_time-start_time),"\n")
                xsol = []
                xsol = pricing.getAttr('x', arc)
                asol = []
                asol = pricing.getAttr('x', customer)
                visit = [0 for j in range(m)]
                succ = [-1 for j in range(m+1)]
                for i in range(m+1):
                    for j in range(m+3):
                        if (i,j) in E and round(xsol[(i,j)]) == 1:
                            succ[i] = j
                nj = succ[m]                 
                while nj < m:
                    visit[nj] = 1
                    nj = succ[nj]
                for j in nodes:
                    if round(xsol[m+1,j]) == 1:
                        nj = j
                        while nj<m:
                            visit[nj] = 1
                            nj = succ[nj]
                flag = True
                for j in nodes:
                    if visit[j] == 0 and round(asol[j]) == 1:
                        flag = False
                        cycle = [j]
                        visit[j] = 1
                        nj = succ[j]
                        while nj != j:
                            cycle.append(nj)
                            visit[nj] = 1
                            nj = succ[nj]
                        pricing.addConstr(quicksum(arc[k,l] for k in set(cycle) for l in set(cycle)-{k}) <= len(cycle)-1)   
            #print(n,pricing.ObjVal,cc,'\n')   
        #add column to master LP
            if(pricing.ObjVal > -0.000001): 
                break
            svalues = [int(asol[i]) for i in range(m)]
            A.append(svalues)
            print('d',pricing.ObjVal,'\n',len(A),'\n')
            c.append(0.7*sum(dist[i][j]*xsol[i,j] for (i,j) in E))
            col = Column()
            for i in range(m):
                col.addTerms(svalues[i], node[i])        
            z[n] = master.addVar(obj=c[n]+v_cost, vtype=GRB.CONTINUOUS, name="x[%d]"%n, column=col) #c[n]+30
            master.update()   
            n += 1
        print(obj_ite,'\n',time_ite,'\n')    
        #Solve the IP model with integrality restrictions on the generated columns
        for j in range(len(A)):
            z[j].setAttr("vtype", GRB.BINARY)
        for i in range(m):
            node[i].setAttr("sense", GRB.EQUAL)
        #if ct != 0:
            #master.addConstr(quicksum(z[j] for j in range(len(A))) <= veh)
            
        master.update()
        master.optimize()
        zsol = []
        zsol= master.getAttr('x', z)  
        end_time = tm.time() 
        print("# Vehicles = "+str(round(sum([zsol[i] for i in range(len(zsol))]))),"\n")
        print("Total cost = "+str(master.ObjVal),"\n")
        print("Time = "+str(end_time-start_time),"\n")
        veh.append(round(sum([zsol[i] for i in range(len(zsol))])))
        cos.append(master.ObjVal-v_cost*round(sum([zsol[i] for i in range(len(zsol))])))
        tim.append(end_time-start_time)
    else:
        obj_ite = []
        time_ite = []
        #master LP
        if m >= 25:
            U = 10
        elif m >= 20:
            U = 15
        elif m >= 15:
            U = 20
        else:
            U = 25
        tot_start_time = tm.time()
        master = Model("VRP")
        master.setParam("OutputFlag",0)
        
        #Initialization: generate initial routes using pseudo-sweep heuristic
        A = [] #set of columns or matrix of master problem
        c = [] #set of cost of each column in A
        s = [0]*m #s stores the route generated
        ft = [] #set of time of first trip
        lt = [] #set of time of last trip
        s[0] = 1
        count = 1
        cap = demand[0] 
        time = 1/40*dist[m][0]
        st_time = 1/40*dist[m][0]
        while count<=m-1:
            time += 1/40*dist[count-1][count]
            cap += demand[count]
            ed_time = 1/40*dist[count][m]
            tot_time = time+ed_time
            if cap<=60 and tot_time<=11 and tot_time-st_time<=10 and time-st_time<=8:
                s[count] = 1
            else:
                A.append(s)
                start = s.index(1)
                end = start+sum(s)-1
                c.append(0.7*(dist[m][start]+sum(dist[i][i+1] for i in range(start,end))+dist[end][m]))
                ft.append(dist[m][start])
                lt.append(dist[end][m])
                s = [0]*m
                s[count] = 1
                cap = demand[count]
                time = 1/40*dist[m][count]
                st_time = 1/40*dist[m][count]
            count += 1
        A.append(s)
        start = s.index(1)
        end = start+sum(s)-1
        c.append(0.7*(dist[m][start]+sum(dist[i][i+1] for i in range(start,end))+dist[end][m]))
        ft.append(dist[m][start])
        lt.append(dist[end][m])
        
        node = {}
        z = {}
        n = len(A)
        for j in range(n):
            z[j] = master.addVar(obj=c[j]+v_cost, vtype=GRB.CONTINUOUS, name="num[%d]"%j) #c[j]+30
        for i in range(m):
            node[i] = master.addConstr(quicksum(A[j][i]*z[j] for j in range(n)) >= 1, name="dem[%d]"%i)
        master.update()  
        
        count = 0
        while 1:
        #resolve master LP
            master.optimize()
            obj_ite.append(master.ObjVal)
            end_time = tm.time()
            time_ite.append(end_time-tot_start_time)
            profit = [node[i].Pi for i in range(m)]    #optimal dual vars 
        
            w = [[inf for j in range(m+1)] for i in range(m+1)]
        #adjust the structure of the graph and the distance
            for i in range(m+1):
                for j in set(range(m+1))-{m,i}:
                    w[i][j] = 0.7*dist[i][j]-profit[j]
            for i in range(len(w)):
                w[i].append(0.7*dist[i][m])
        
        #pricing problem: random coloring
            for num in range(10):
                #start_time = tm.time()
                #V = range(m+2) #all the vertexes including the dummy depots
                r = random.random()
                if r<0.8:
                    num_col = 6
                elif r<0.9:
                    num_col = 5
                else:
                    num_col = 4
                while 2:
                    Color = [random.randint(0,num_col-1) for i in range(m)]
                    if num_col == len(set(Color)):
                        break
                #Color.append(Color[-1])
                color = list(range(2))
                for i in range(1,num_col):
                    l = list(range(2))
                    color = list(product(color,l))
                    for j in range(len(color)):
                        if i == 1:
                            tp1 = (color[j][0],)
                        else:
                            tp1 = color[j][0]   
                        tp2 = (color[j][1],)
                        color[j] = tp1+tp2
                S = list(product(color,range(61)))
                for j in range(len(S)):
                    tp1 = S[j][0]
                    tp2 = (S[j][1],)
                    S[j] = tp1+tp2
                
                T = [[inf for j in S] for i in range(m+2)]
                P = [[[inf,inf] for j in S] for i in range(m+2)]
                for j in range(61): #initialization
                    T[m][j] = 0
          
                for j in range(1,len(S)): #ignore [0,0,0,0,0,0]
                    for V in range(m):
                        if S[j][Color[V]] == 1: #color of v in s
                            pre = [inf,inf]
                            opt = inf
                            for p in range(m+1):
                                ps = [i for i in S[j]]
                                if V != m+1:
                                    ps[Color[V]] = 0
                                ps[-1] -= int(demand[V])
                                if ps[-1] >= 0 and T[p][S.index(tuple(ps))]+w[p][V] < opt:
                                    opt = T[p][S.index(tuple(ps))]+w[p][V]
                                    pre = [p,S.index(tuple(ps))]
                            T[V][j] = opt
                            P[V][j] = pre
                s = [1 for i in range(num_col)]
                pre = [inf,inf]
                opt = inf
                for V in range(m):
                    for cap in range(61):
                        tp = tuple(s+[cap])
                        if T[V][S.index(tp)]+w[V][m+1] < opt:
                            opt = T[V][S.index(tp)]+w[V][m+1]
                            pre = [V,S.index(tp)]
                T[m+1][len(S)-1] = opt
                P[m+1][len(S)-1] = pre
                if T[m+1][len(S)-1] < -0.000001:
                    print(T[m+1][len(S)-1])
                    count = 0
                    pre = P[m+1][len(S)-1]
                    path = [m+1]
                    s = [0 for i in range(num_col)]
                    while 1:
                        path.append(pre[0])
                        tp = list(S[pre[1]])
                        tp.remove(tp[-1])
                        if pre[0] == m and tp == s:
                            break
                        pre = P[pre[0]][pre[1]]
                    path.reverse()
                    print(path)
                    #end_time = tm.time() 
                    #print("Time = "+str(end_time-start_time),"\n")
                    total_dist = sum(dist[path[i]][path[i+1]] for i in range(len(path)-2))+dist[path[-2]][m]
                    if total_dist < 440:
                        coln = [0 for i in range(m)]
                        for i in range(1,len(path)-1):
                            coln[path[i]] = 1
                        if coln in A:
                            continue
                        A.append(coln)
                        cost = 0.7*total_dist
                        c.append(cost)
                        ft.append(dist[path[0]][path[1]])
                        lt.append(dist[path[-2]][path[0]])
                        col = Column()
                        for i in range(m):
                            col.addTerms(coln[i], node[i])        
                        z[n] = master.addVar(obj=c[n]+v_cost, vtype=GRB.CONTINUOUS, name="x[%d]"%n, column=col) 
                        master.update()   
                        n += 1
                else:
                    count += 1
                    print(T[m+1][len(S)-1],'\n')
            break       
            if count > 5 or n > U:
                break
        print(obj_ite,'\n',time_ite,'\n')      
        #solve a new IP with columns generated before
        v = ceil(sum(demand)/60) #upper bound of # vehicles
        t = len(A)
        ut = list(product(range(v),range(t)))
        tour_index = []
        for i in range(m):
            t_index = []
            for j in range(t):
                if A[j][i] == 1:
                    t_index.append(j)
            tour_index.append(t_index)
        IPmod = Model("Final")
        IPmod.setParam("OutputFlag",0)
        y = IPmod.addVars(range(t), vtype=GRB.BINARY, name="subtour") #30
        z = IPmod.addVars(range(v), vtype=GRB.BINARY,name="vehicle") #2
        x = IPmod.addVars(ut, vtype=GRB.BINARY,name="ut") #60
        F = IPmod.addVars(ut, vtype=GRB.BINARY,name="F") #60
        L = IPmod.addVars(ut, vtype=GRB.BINARY,name="L") #60
        IPmod.setObjective(quicksum(c[tau]*x[u,tau] for (u,tau) in ut)+v_cost*quicksum(z[i] for i in range(v)), GRB.MINIMIZE)
        for tau in range(t):
            IPmod.addConstr(quicksum(x[u,tau] for u in range(v)) == y[tau], name="tour"+str(tau))
        for u in range(v):
            IPmod.addConstr(quicksum(F[u,tau] for tau in range(t)) <= z[u], name="F"+str(u)+str(tau))
            IPmod.addConstr(quicksum(L[u,tau] for tau in range(t)) <= z[u], name="L"+str(u)+str(tau))
            IPmod.addConstr(quicksum(c[tau]*x[u,tau] for tau in range(t)) <= 440, name="vehicle_time"+str(u)+str(tau))
            IPmod.addConstr(quicksum(c[tau]*x[u,tau] for tau in range(t))-quicksum(ft[tau]*F[u,tau] for tau in range(t)) <= 400, name="vehicle_time_1"+str(u)+str(tau))
            IPmod.addConstr(quicksum(c[tau]*x[u,tau] for tau in range(t))-quicksum(ft[tau]*F[u,tau] for tau in range(t))-quicksum(lt[tau]*L[u,tau] for tau in range(t)) <= 320, name="vehicle_time_2"+str(u)+str(tau))
        
        for (u,tau) in ut:
            IPmod.addConstr(x[u,tau] <= z[u], name="vehicle"+str(u)+str(tau))
            IPmod.addConstr(F[u,tau] <= x[u,tau], name="first"+str(u)+str(tau))
            IPmod.addConstr(L[u,tau] <= x[u,tau], name="last"+str(u)+str(tau))
        for i in range(m):
            IPmod.addConstr(quicksum(y[tau] for tau in tour_index[i]) >= 1, name="customer"+str(i))
        IPmod.optimize()
        zsol = []
        zsol = IPmod.getAttr('x', z)  
        
        print("# Vehicles = "+str(round(sum([zsol[i] for i in range(len(zsol))]))),"\n")
        print("Total cost = "+str(IPmod.ObjVal),"\n")
        tot_end_time = tm.time() 
        print("Time = "+str(tot_end_time-tot_start_time),"\n")
        veh.append(round(sum([zsol[i] for i in range(len(zsol))])))
        cos.append(IPmod.ObjVal-v_cost*round(sum([zsol[i] for i in range(len(zsol))])))
        tim.append(tot_end_time-tot_start_time)
    #ct += 1        
print("TIME:",tim)
print("COST:",cos)
print("VEHICLES:",max(veh))
print("TOTAL_TIME:",sum(tim))
print("TOTAL_COST:",sum(cos)*365/4)
