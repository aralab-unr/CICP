# CICP
This is the basic implmentation of Correntropy ICP.
How to use it.

1) Open terminal and compile PCL present in the folder libraries/pcl using the basic instructions of pcl. This pcl contains our implmentation of Correntropy ICP.
2) Open another terminal and navigate to mcc-icp/MCC_ICP_PCL/src directory.
3) COmpile the mcc-icp using the commands:
      cmake ..
      make 
4) The source code to run a basic sample is present in mcc_icp.cpp.
5) IN terminal type the following to run the mcc-cip.
       ./mcc-icp bunny01.ocd 1
       
