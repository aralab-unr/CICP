# CICP
This is the basic implmentation of Correntropy ICP and below are the instructions on how to use it.

1) Open terminal and compile PCL present in the folder libraries/pcl using the basic instructions of pcl. This pcl contains our implmentation of Correntropy ICP.
2) Open another terminal and navigate to mcc-icp/MCC_ICP_PCL/src directory.
3) Compile the mcc-icp using the commands:
      cmake ..
      make 
4) The source code to run a basic sample is present in mcc_icp.cpp.
5) In the terminal type the following to run the mcc-cip.
       ./mcc-icp bunny01.pcd 1
      
you can change Line 137 in the mcc-icp.cpp file change the CorrentropySVD to any other estimation procedure.
       
