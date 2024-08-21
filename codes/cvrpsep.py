# -*- coding: utf-8 -*-
"""
Created on Tue Jul 25 10:53:17 2023

@author: FULIANG WU
"""

from sys import exit
import ctypes
import platform
# Determine the operating system
system = platform.system()

# =============================================================================
# cvrpsep
# =============================================================================
# Load the shared library (change 

if system == "Darwin":  # macOS
    try:
        mylib = ctypes.CDLL('./sep/mylib.dylib')  # Load .dylib on macOS
    except OSError:
        print("Error loading the .dylib library.")
        exit(1)
elif system == "Linux":
    try:
        mylib = ctypes.CDLL('./sep/mylib.so')  # Load .so on Linux
    except OSError:
        print("Error loading the .so library.")
        exit(1)
else:
    print(f"Unsupported operating system: {system}")
    exit(1)


def Sep(NoOfCus, Dem, CAP, Edges):
    
    NoOfCus = int(NoOfCus)
    demLis = [0] + [int(ele) for ele in Dem]
    
    edgTaiLis = []
    for ele in Edges:
        if ele[0] != 0:
            edgTaiLis.append(ele[0])
        else:
            edgTaiLis.append(NoOfCus + 1) # The depot, since there is no 0 in cvrpsep
            
    edgHeaLis = [int(ele[1]) for ele in Edges]
    edgLis = [float(1) for ele in Edges]
    NoOfEdges = len(Edges)
    MaxNoOfCuts = int(NoOfCus**2)
    
    cut = (ctypes.c_int * MaxNoOfCuts)(int(0))
    Demand = (ctypes.c_int * int(NoOfCus+2))(*demLis)
    EdgeTail = (ctypes.c_int * (NoOfEdges+1))(*edgTaiLis)  # Convert to ctypes array
    EdgeHead = (ctypes.c_int * (NoOfEdges+1))(*edgHeaLis)  # Convert to ctypes array
    EdgeX = (ctypes.c_double * (NoOfEdges+1))(*edgLis)  # Convert to ctypes array
    
    # Call the C++ function
    NoOfCuts = mylib.sepCut(ctypes.c_int(NoOfCus), Demand, ctypes.c_int(int(CAP)), ctypes.c_int(NoOfEdges), EdgeTail, EdgeHead, EdgeX, ctypes.c_int(MaxNoOfCuts), cut)

    # Print the results
    cuts = []
    nCus = NoOfCus
    for i in range(NoOfCuts):
        cuts.append([ele for ele in list(cut[i*nCus: (i+1) * nCus]) if ele != 0])
    
    # if len(cuts) >= 1:
        # print(list(cut))
    
    return cuts











