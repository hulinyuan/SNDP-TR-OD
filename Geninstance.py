# -*- coding: utf-8 -*-
"""
Created on Mon Aug 23 21:17:22 2021

@author: 胡琳苑
"""
import random
import csv

class inpm:
    terminal=[]
    connection=[]
    demand=[] 
    t = 0
    
class vehicle:
    fixed_cost=0
    capacity =0
    unitcost=0
    drivercost=0
    zone=0
    
class Lane:
    one = 0
    ano = 0
    time = 0
    zone = 0
    unitcost=0

inf=10000#inf
 
def Dijkstra_all_minpath(start,matrix):
    length=len(matrix)#该图的节点数
    path_array=[]
    temp_array=[]
    path_array.extend(matrix[start])#深复制
    temp_array.extend(matrix[start])#深复制
    temp_array[start] = inf#临时数组会把处理过的节点的值变成inf，表示不是最小权值的节点了
    already_traversal=[start]#start已处理
    path_parent=[start]*length#用于画路径，记录此路径中该节点的父节点
    while(len(already_traversal)<length):
        i= temp_array.index(min(temp_array))#找最小权值的节点的坐标
        temp_array[i]=inf
        path=[]#用于画路径
        path.append(str(i))
        k=i
        while(path_parent[k]!=start):#找该节点的父节点添加到path，直到父节点是start
            path.append(str(path_parent[k]))
            k=path_parent[k]
        path.append(str(start))
        path.reverse()#path反序产生路径
        print(str(i)+':','->'.join(path))#打印路径
        already_traversal.append(i)#该索引已经处理了
        for j in range(length):#这个不用多说了吧
            if j not in already_traversal:
                if (path_array[i]+matrix[i][j])<path_array[j]:
                    path_array[j] = temp_array[j] =path_array[i]+matrix[i][j]
                    path_parent[j]=i#说明父节点是i
                    
    return path_array


pm1=inpm()
pm1.t =7
pm1.terminal =[10,10,10]
pm1.connection = [1,1,1]
pm1.demand=[100,50]
num = 5
filename = f'ins{num}.csv'

# 长途车型
vown=list()
vlong = list()
v1 = vehicle()
v1.fixed_cost = 50000 
v1.capacity =290000 #29t 53chi
v1.unitcost =0.23
v1.drivercost = 1000
vlong.append(v1) 

v2 = vehicle()
v2.fixed_cost = 30000 
v2.capacity =180000 #45chi
v2.unitcost =0.23
v2.drivercost = 1000
vlong.append(v2)
vown.append(vlong)

vshort =list()
# 短途车型10,8,5,3
v3 =vehicle()
v3.fixed_cost =10000
v3.capacity =10000
v3.unitcost =0.20
v3.drivercost  =700
vshort.append(v3)

v4=vehicle()
v4.fixed_cost=8000
v4.capacity =8000
v4.unitcost =0.16
v4.drivercost  =500
vshort.append(v4)

v5=vehicle()
v5.fixed_cost=5000
v5.capacity =5000
v5.unitcost =0.12
v5.drivercost  = 400
vshort.append(v5)

v6=vehicle()
v6.fixed_cost = 3000
v6.capacity = 3000
v6.unitcost = 0.10
v6.drivercost = 300
# vshort.append(v6)
vown.append(vshort)


vdown=list()
vlong = list()
s1 = vehicle()
s1.fixed_cost = 50000 
s1.capacity =290000 #29t 53chi
s1.unitcost =0
s1.drivercost = 2000
vlong.append(s1)

s2 = vehicle()
s2.fixed_cost = 30000 
s2.capacity = 180000 #45chi
s2.unitcost = 0
s2.drivercost = 2000
vlong.append(s2)
vdown.append(vlong)


vshort=list()
s3 =vehicle()
s3.fixed_cost =10000
s3.capacity =10000
s3.unitcost =0
s3.drivercost  = 1000
vshort.append(s3)

s4=vehicle()
s4.fixed_cost=8000
s4.capacity =8000
s4.unitcost =0
s4.drivercost  = 800
vshort.append(s4)

s5=vehicle()
s5.fixed_cost=5000
s5.capacity =5000
s5.unitcost =0
s5.drivercost  =700
vshort.append(s5)

s6=vehicle()
s6.fixed_cost =3000
s6.capacity = 3000
s6.unitcost =0
s6.drivercost  = 600
#vshort.append(s6)
vdown.append(vshort)

tms = list()
zone =list()
num = 0
for z in range(len(pm1.terminal)):
    lz_list =list()
    for l in range(pm1.terminal[z]):
        tz=list()
        tz.append(num)
        tz.append(z)
        tms.append(tz)
        lz_list.append(num)
        num = num+1
    zone.append(lz_list)

vehiclelist=list()
count_v =0
for z in range(len(pm1.terminal)):
    
    for vls in vown:
        number = random.randint(1,1)
        for n in range(number):
            v_z =list()
            v=random.choice(vls)
            count_v=count_v+1
            v_z.append(count_v)
            v_z.append(z)
            v_z.append(v.capacity)
            v_z.append(v.fixed_cost)
            v_z.append(v.unitcost)
            v_z.append(v.drivercost)
            vehiclelist.append(v_z)
                
    for vls in vdown:
        number = random.randint(1,1)
        for n in range(number):
            v_z =list()
            v=random.choice(vls)
            count_v=count_v+1
            v_z.append(count_v)
            v_z.append(z)
            v_z.append(v.capacity)
            v_z.append(v.fixed_cost)
            v_z.append(v.unitcost)
            v_z.append(v.drivercost)
            vehiclelist.append(v_z)

# 构建物理网络
# 构建区域内连接的通道
lane_rec = list()
lane_list =list()
z_num=0
for z in zone:
    lane_z =list()
    for l1 in range(len(z)-1):
        for l2 in range(l1+1,len(z)):
            lane =Lane()
            l_list=list()
            if z[l1]!=z[l2]:
                lane.one=z[l1]
                lane.ano=z[l2]
                lane.time=random.randint(1,2)
                lane.unitcost=round(random.uniform(0.1,0.128),2)
                lane.zone=[z_num]
                lane_list.append(lane)     
                l_list.append(z[l1])
                l_list.append(z[l2])
                l_list.append(z_num)
                l_list.append(lane.time)
                l_list.append(lane.unitcost)
                lane_rec.append(l_list)
  
    z_num=z_num+1

# 构建连接区域的通道
connect_l =list()
cnum =0
for z1 in range(len(zone)-1):
    for z2 in range(z1+1,len(zone)):
        z1_con = list()
        for c_num in range(pm1.connection[cnum]):
            cn = random.choice(zone[z1])
            z1_con.append(cn)
            for l in zone[z2]:
                lane =Lane()
                l_list=list()
                lane.one=cn
                lane.ano=l
                lane.time=random.randint(1,2)
                lane.unitcost=round(random.uniform(0.10,0.128),2)
                lane.zone=z2
                lane_list.append(lane)    
                l_list.append(cn)
                l_list.append(l)
                l_list.append(z2)
                l_list.append(lane.time)
                l_list.append(lane.unitcost)
                lane_rec.append(l_list)   
        connect_l.append(z1_con)
        cnum = cnum + 1
    


travel_matrix = list()

for l in range(len(tms)):
    row = [10000]*len(tms)
    row[l] = 10000
    for lane in lane_list:
        if (lane.one == l):
            row[lane.ano]=lane.time
        if(lane.ano ==l):
            row[lane.one]=lane.time
    travel_matrix.append(row)

#领接矩阵
travel_minpath = list()
for l in range(len(tms)):
    travel_minpath.append(Dijkstra_all_minpath(l,travel_matrix))

# 构建需求
Demandlist = list()
for d in range(pm1.demand[0]):
    print(d)
    perdemand=list()
    start_lz = random.choice(tms)
    end_lz = random.choice(tms)
    Trtime = travel_minpath[start_lz[0]][end_lz[0]]+random.randint(0,1)#计算两个站点间的运输时间
    perdemand.append(start_lz[0])
    start_t = random.randint(0,pm1.t-1) #得到需求产生时间
    perdemand.append(start_t)
    arrive_t = start_t + Trtime
    while((start_lz[0]==end_lz[0])or((arrive_t>pm1.t))) :
        end_lz = random.choice(tms) #得到起点和终点
        Trtime = travel_minpath[start_lz[0]][end_lz[0]]+random.randint(0,1)#计算两个站点间的运输时间
        arrive_t = start_t + Trtime
    
   
    perdemand.append(end_lz[0])
    perdemand.append(arrive_t%pm1.t)
    volume = random.randint(100,3000)# 货运量kg
    perdemand.append(volume)
    revenue = round(1.28*volume,2)
    perdemand.append(revenue)
    perdemand.append(0)
    Demandlist.append(perdemand)

for d in range(pm1.demand[1]):
    print(d)
    perdemand=list()
    start_lz = random.choice(tms)
    end_lz = random.choice(tms) 
    Trtime = travel_minpath[start_lz[0]][end_lz[0]]+random.randint(0,1)#计算两个站点间的运输时间
    perdemand.append(start_lz[0])
    start_t = random.randint(0,pm1.t-1) #得到需求产生时间
    perdemand.append(start_t)
    arrive_t = start_t + Trtime
    while((start_lz[0]==end_lz[0])or(arrive_t>pm1.t)):
        end_lz = random.choice(tms) #得到起点和终点
        Trtime = travel_minpath[start_lz[0]][end_lz[0]]+random.randint(0,1)#计算两个站点间的运输时间
        arrive_t = start_t + Trtime
        

    perdemand.append(end_lz[0])
    perdemand.append(arrive_t%pm1.t)
    volume = random.randint(100,3000)# 货运量kg
    perdemand.append(volume)
    revenue = round(1.28*volume,2)
    perdemand.append(revenue)
    perdemand.append(1)
    Demandlist.append(perdemand)

#输出csv文件
f = open(filename,'w',encoding='utf-8',newline="")
csv_writer = csv.writer(f)
pm1.t
csv_writer.writerow(['Terminal','zone'])
for tz in tms:
    csv_writer.writerow(tz)
    
csv_writer.writerow(['Connect','zone'])
for cl in connect_l:
    csv_writer.writerow(cl)

csv_writer.writerow(['vehicle','zone','capacity','fixed_cost','unitcost','drivercost'])
for vc in vehiclelist:
    csv_writer.writerow(vc)
    
csv_writer.writerow(['Lane','la2','zone','time','unitcost'])
for lane in lane_rec:
    csv_writer.writerow(lane)
    
#csv_writer.writerow(['Matrix','traveltime']) 
#for row in travel_matrix:
#    csv_writer.writerow(row)
    
csv_writer.writerow(['Demand','st','el','et','volume','revenue','type'])
for demand in Demandlist:
    csv_writer.writerow(demand)
    
csv_writer.writerow(['Period'])
csv_writer.writerow([pm1.t])

f.close()



