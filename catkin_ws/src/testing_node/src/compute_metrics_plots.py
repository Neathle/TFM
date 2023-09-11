#! /usr/bin/env python
#Script that process text files with pose data to compute metrics and export plots. 
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import rospy
import os

def ang_rango(ang):
    if ang > np.pi:
        ang = ang - 2 * np.pi
        return ang_rango(ang)  # Return the result of the recursive call
    if ang < -np.pi:
        ang = ang + 2 * np.pi
        return ang_rango(ang)  # Return the result of the recursive call
    return ang

path         = rospy.get_param('/testing_node_metrics/data_path')
scenario = rospy.get_param('/testing_node_metrics/scenario')
laser = "Virtual Laser"
print("data path is", path)

#scenario = "pare"
#path = "/home/fer/Desktop/Pruebas_amcl/params_tunning_fer/"+scenario+"/"
#path = "/home/fer/Desktop/Pruebas_amcl/params_tunning_fer/"+scenario+"_depth/"
#laser = "Virtual Laser" 
                
files = os.listdir(path)
ate_all = []
filenames = []
for file in files:
    if file.endswith( '.txt'):
        filenames.append(file) #append in list all txt files located in the given path
num_files = len(filenames)/3
num_files = int(num_files)
print( "num_files is: ", num_files)

for file in range(1,num_files+1):
    df_amcl = pd.read_csv(path+"data_amcl_"+str(scenario)+"_test"+str(file)+".txt")
    df_gdth = pd.read_csv(path+"data_gdtr_"+str(scenario)+"_test"+str(file)+".txt") 
    df_odom = pd.read_csv(path+"data_odom_"+str(scenario)+"_test"+str(file)+".txt")

    df_gdth = df_gdth.rename({'TimeStamp': 'gdth_Timestamp'}, axis='columns') #rename tstamp column to avoid confusions when concat
    df_gdth = df_gdth.rename({'Seq': 'gdth_Seq'}, axis='columns') #rename tstamp column to avoid confusions when concat

    
    df_ate = pd.concat([df_amcl,df_gdth], axis=1) #concat df amcl left, gdth right
    #df_ate = df_ate.drop(["Xamcl_cov","Yamcl_cov","Thamcl_cov"], axis=1) #drop unused columns for the moment
    df_ate.to_csv(path+"DEBUG1.txt", sep='\t')
    len_init = len(df_ate)
    
    df_ate.to_csv(path+"DEBUG2.txt", sep='\t')
    print( "File num is: ", file, "len inicial df", len_init, "len depurada df", len(df_ate) )
    
    #tengo que alinear los datos respecto a los tstamps antes de hacer drop a los empty rows
    #al df del gtruth irle appending los datos del amcl correspondientes
    for i in range(0,len(df_amcl)):
        for j in range(0,len(df_gdth)):
            if df_amcl["TimeStamp"][i] < df_gdth["gdth_Timestamp"][j]:
                df_ate["Xgt"][i] =  df_gdth["Xgt"][j]
                df_ate["Ygt"][i] =  df_gdth["Ygt"][j]
                df_ate["Thgt"][i] = df_gdth["Thgt"][j]
                df_ate["gdth_Timestamp"][i] = df_gdth["gdth_Timestamp"][j]
                df_ate["gdth_Seq"][i] = df_gdth["gdth_Seq"][j]
                #print("el punto de amcl ", i, "hace match con el de gtruth ", j)
                break

    df_ate = df_ate[(~df_ate.isnull()).all(axis=1)]

    #convertir angulos a radianes
    df_ate["Thgt"] = df_ate["Thgt"] * (np.pi/180)
    df_ate["Thacml"] = df_ate["Thacml"] * (np.pi/180)
    
    #normalizar angulo entre intervalo [-pi,pi]
    for i in range(0,len(df_amcl)):
        df_ate["Thgt"][i]   = ang_rango(df_ate["Thgt"][i])
        df_ate["Thacml"][i] = ang_rango(df_ate["Thacml"][i])
    
    #normalizar angulo entre intervalo [-pi,pi]
    for i in range(0,len(df_amcl)):
        if df_ate["Thgt"][i] < -np.pi:
            ang=2*PI+ang;
            AngRango(ang);
        if df_ate["Thgt"][i] > np.pi:
            ang=ang-2*PI;
            AngRango(ang);
    
    #Compute pose errors x_error, y_error, th_error, traj_error
    df_ate["x_error"]  = df_ate["Xgt"]   -df_ate["Xamcl"]  
    df_ate["y_error"]  = df_ate["Ygt"]   -df_ate["Yamcl"]  
    df_ate["th_error"] = df_ate["Thgt"] -df_ate["Thacml"]
    
    for i in range(0,len(df_amcl)):
        df_ate["th_error"][i]   = ang_rango(df_ate["th_error"][i])
    
    df_ate["traj_error"] =(np.sqrt( (df_ate["x_error"]*df_ate["x_error"]) + (df_ate["y_error"]*df_ate["y_error"]) ) )
    df_ate["traj_error2"]=df_ate["traj_error"] * df_ate["traj_error"]
    
    #Compute metrics for trajectory error, min, max, mean, std, ate
    minT = round(df_ate["traj_error"].min() ,4)
    maxT = round(df_ate["traj_error"].max() ,4)
    mean = round(df_ate["traj_error"].mean(),4)
    std     = round(df_ate["traj_error"].std() ,4)
    ate     = np.sqrt(df_ate["traj_error2"].mean())
    ate     = round(ate,4)#ATE_RSME = sqrt[mean(error_trayectoria al 2)]    
    print( "min is ", minT, "max is ", maxT, "mean is ", mean, "std is ", std, "ATE is ", ate )
    
    ######
    #PLOT1 - trajectory error vs. time /trajectory in map (odom, groundtruth, amcl)
    ######
    fig, ax = plt.subplots(1,2)
    fig.set_size_inches(18,8)
    fig.suptitle(" AMCL Test "+str(file)+" "+laser, fontsize=24)
    
    ax[0].set_title("Trajectory")
    for i in range(0,len(df_ate)):
        x1 = float(df_ate["Xamcl"][i])
        x2 = float(df_ate["Xgt"][i])
        y1 = float(df_ate["Yamcl"][i])
        y2 = float(df_ate["Ygt"][i]) 
        ax[0].plot([x1,x2],[y1,y2], 'k-', linewidth=0.5)
    ax[0].plot(df_ate["Xamcl"],  df_ate["Yamcl"], label="AMCL")
    ax[0].plot(df_ate["Xgt"],   df_ate["Ygt"],  label="Groundtruth")
    #ax[0].scatter(df_gdth["Xgt"],   df_gdth["Ygt"],  label="Groundtruth")
    #ax[0].plot(df_odom["Xodom"], df_odom["Yodom"],label="Odometry")
    ax[0].legend(loc='upper left')
    ax[0].set_xlabel("X position")
    ax[0].set_ylabel("Y position")
    
    ax[1].set_title("Trajectory Error")
    ax[1].plot(df_ate["traj_error"]) 
    ax[1].text(30,1, "ATE: %s\nMean:%s\nStd: %s\nMax: %s\nMin: %s\n" %(ate, mean, std, maxT, minT), fontsize = 10)
    ax[1].set_xlabel("Poses")
    ax[1].set_ylabel("Error")
    ax[1].set_ylim([0, 8])
       
    fig.savefig(path+"amcl_test"+str(file)+"_traj_error.png")
    
    ######
    #PLOT2 - pose error/covariance x,y,th vs. time
    ######
    fig, ax = plt.subplots(3,2)
    fig.set_size_inches(20,16)
    fig.suptitle(" AMCL Test "+str(file)+" "+ laser, fontsize=24)
    
    ax[0,0].plot( df_ate["x_error"]) 
    ax[0,0].plot(df_ate["x_error"]) 
    ax[0,0].set_title("Pose Error in X")
    ax[1,0].plot( df_ate["y_error"]) 
    ax[1,0].set_title("Pose Error in Y")
    ax[2,0].plot( df_ate["th_error"]) 
    ax[2,0].set_title("Pose Error in Th")
    ax[0,1].plot( np.sqrt(df_ate["Xamcl_cov"]))  
    ax[0,1].set_title("Std dev in X")
    ax[1,1].plot( np.sqrt(df_ate["Yamcl_cov"]))
    ax[1,1].set_title("Std dev  in Y")
    ax[2,1].plot( np.sqrt(df_ate["Thamcl_cov"])) 
    ax[2,1].set_title("Std dev  in Th")
    
    fig.savefig(path+"amcl_test"+str(file)+"_pose_error_cov.png")
    ate_all.append([ate])
   


######
#PLOT3 - all tests statistics
######
df_ate_all   = pd.DataFrame(ate_all, columns=["ATE"])
mean_ate_all = round(df_ate_all["ATE"].mean(),4)
#print( df_ate_all )
#a-trayectorias encimadas, traj. error encimados, ATE barras
fig, ax = plt.subplots(1,1)
fig.set_size_inches(9,6)
fig.suptitle(" AMCL Tests Summary " + laser, fontsize=24)

threshold = mean_ate_all
df_ate_all['ATE'].plot.bar(x='Test',y='ATE') 
ax.set_title("ATE")
ax.plot([0., len(df_ate_all)], [threshold, threshold], "k--")
ax.text(3,3.1, "Mean:%s\n" % (mean_ate_all), fontsize = 10)
ax.set_ylim([0, 5])

fig.savefig(path+"amcl_tests_summary.png")
