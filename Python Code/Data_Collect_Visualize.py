#Rebeca Gaston 
#2DX3 Final Project Data Collection and Visualization Code

import serial
import math
import numpy as np
import open3d as o3d                       

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
s = serial.Serial('COM3', 115200, timeout = 10)
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()


# recieve measurements from UART of MCU
file = open("datainfo.xyz", "w")         #open file in write mode for data collection (clears any prev data)
file.close()
scanNum = 3                               #desired number of scans   
x = 0                                     #fixed iteration param
x_displacement = 100                      #desired x displacement per scan (mm)
iteration = 1                             #fixed iteration param
total_iterations = 32                     #chosen param based on stepper angle
passes = 0                                #fixed iteration param
deg = 11.25                               #stepper motor angle (degrees) 
while 1:                                  #main loop
    b = (s.readline()).decode()             #reads bytes --> convert to string
    b = b[0:-2]                             #remove the last two unwanted digits from string
    if (b.isdigit() == True):               #only runs if we are getting a digit input communicated
        b = int(b)
        angle = iteration*(deg*math.pi/180)
        y = b*math.cos(angle)
        z = b*math.sin(angle) 
        print("b = ", b)
        print("Distance Measurment: ")
        print("[", x, ", ", y,  ", ",  z, "]")
        file = open("datainfo.xyz", "a")             #opens file to append data 
        file.write('{} {} {}\n'.format(x,y,z))
        file.close()                            #close file
        iteration +=1
    if (iteration == (total_iterations+1)):                   #checks if next iteration is a new full turn 
        if (passes == (scanNum-1)):                    #checks if we are done all required measurements -11 for now cause testing 1 scan noly 
            break                           #break out of loop 
        else:
            passes += 1 
            x += x_displacement             #update x distance(mm)                    
            iteration = 1                   #reset iteration 
#close the port
print("Closing: " + s.name)
s.close()

#VISUALIZATION CODE

if __name__ == "__main__":
    #Remember the goals of modularization
    #   -- smaller problems, reuse, validation, debugging
    #To simulate the data from the sensor lets create a new file with test data
    #Read the test data in from the file we created        
    print("Read in the point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("datainfo.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Add some lines to connect the vertices
    #   For creating a lineset we will need to tell the packahe which vertices need connected
    #   Remember each vertex actually contains one x,y,z coordinate
    #Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0,scanNum*total_iterations):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = [] 
    for x in range(0,scanNum*total_iterations,total_iterations):
        for i in range(x, x+total_iterations-1,1):
            lines.append([yz_slice_vertex[i], yz_slice_vertex[i+1]])
        lines.append([yz_slice_vertex[x+total_iterations-1], yz_slice_vertex[x]])
        

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    topindex = 0
    #Define coordinates to connect lines between current and next yz slice        
    for x in range(0,scanNum-1,1):
        for i in range(topindex, topindex+total_iterations,1):
            lines.append([yz_slice_vertex[i], yz_slice_vertex[i+total_iterations]])
        topindex += total_iterations
    
    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
                                    

