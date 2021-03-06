/*******************************************************************************
Vendor: Xilinx 
Associated Filename: test-cl.c
Purpose: OpenCL Host Code for Matrix Multiply Example
Revision History: July 1, 2013 - initial release
                                                
*******************************************************************************
Copyright (C) 2013 XILINX, Inc.

This file contains confidential and proprietary information of Xilinx, Inc. and 
is protected under U.S. and international copyright and other intellectual 
property laws.

DISCLAIMER
This disclaimer is not a license and does not grant any rights to the materials 
distributed herewith. Except as otherwise provided in a valid license issued to 
you by Xilinx, and to the maximum extent permitted by applicable law: 
(1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS, AND XILINX 
HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, 
INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR 
FITNESS FOR ANY PARTICULAR PURPOSE; and (2) Xilinx shall not be liable (whether 
in contract or tort, including negligence, or under any other theory of 
liability) for any loss or damage of any kind or nature related to, arising under 
or in connection with these materials, including for any direct, or any indirect, 
special, incidental, or consequential loss or damage (including loss of data, 
profits, goodwill, or any type of loss or damage suffered as a result of any 
action brought by a third party) even if such damage or loss was reasonably 
foreseeable or Xilinx had been advised of the possibility of the same.

CRITICAL APPLICATIONS
Xilinx products are not designed or intended to be fail-safe, or for use in any 
application requiring fail-safe performance, such as life-support or safety 
devices or systems, Class III medical devices, nuclear facilities, applications 
related to the deployment of airbags, or any other applications that could lead 
to death, personal injury, or severe property or environmental damage 
(individually and collectively, "Critical Applications"). Customer assumes the 
sole risk and liability of any use of Xilinx products in Critical Applications, 
subject only to applicable laws and regulations governing limitations on product 
liability. 

THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT 
ALL TIMES.

*******************************************************************************/
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <CL/opencl.h>

#include "my_socket.h"
#include "my_timer.h"

#define DATA_SIZE 51200
#define TOTAL_TASK_NUMS 1024
#define RESULT_SIZE TOTAL_TASK_NUMS*4


////////////////////////////////////////////////////////////////////////////////

int
load_file_to_memory(const char *filename, char **result)
{ 
  int size = 0;
  FILE *f = fopen(filename, "rb");
  if (f == NULL) 
  { 
    *result = NULL;
    return -1; // -1 means file opening fail 
  } 
  fseek(f, 0, SEEK_END);
  size = ftell(f);
  fseek(f, 0, SEEK_SET);
  *result = (char *)malloc(size+1);
  if (size != fread(*result, sizeof(char), size, f)) 
  { 
    free(*result);
    return -2; // -2 means file reading fail 
  } 
  fclose(f);
  (*result)[size] = 0;
  return size;
}

int main(int argc, char** argv)
{
  int err;                            // error code returned from api calls
     
  int a[DATA_SIZE];                   // original data set given to device
  int results[RESULT_SIZE];             // results returned from device
  unsigned int correct;               // number of correct results returned

  size_t global[2];                   // global domain size for our calculation
  size_t local[2];                    // local domain size for our calculation

  cl_platform_id platform_id;         // platform id
  cl_device_id device_id;             // compute device id 
  cl_context context;                 // compute context
  cl_command_queue commands;          // compute command queue
  cl_program program;                 // compute program
  cl_kernel kernel;                   // compute kernel
   
  char cl_platform_vendor[1001];
  char cl_platform_name[1001];
   
  cl_mem input_a;                     // device memory used for the input array
  //cl_mem input_b;                     // device memory used for the input array
  cl_mem output;                      // device memory used for the output array
  int inc;
  double t_start, t_end;

  if (argc != 2){
    printf("%s <inputfile>\n", argv[0]);
    return EXIT_FAILURE;
  }

  // Connect to first platform
  //
  err = clGetPlatformIDs(1,&platform_id,NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: Failed to find an OpenCL platform!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  err = clGetPlatformInfo(platform_id,CL_PLATFORM_VENDOR,1000,(void *)cl_platform_vendor,NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: clGetPlatformInfo(CL_PLATFORM_VENDOR) failed!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  printf("CL_PLATFORM_VENDOR %s\n",cl_platform_vendor);
  err = clGetPlatformInfo(platform_id,CL_PLATFORM_NAME,1000,(void *)cl_platform_name,NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: clGetPlatformInfo(CL_PLATFORM_NAME) failed!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  printf("CL_PLATFORM_NAME %s\n",cl_platform_name);
 
  // Connect to a compute device
  //
  int fpga = 0;
#if defined (FPGA_DEVICE)
  fpga = 1;
#endif
  err = clGetDeviceIDs(platform_id, fpga ? CL_DEVICE_TYPE_ACCELERATOR : CL_DEVICE_TYPE_CPU,
                       1, &device_id, NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: Failed to create a device group!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  
  // Create a compute context 
  //
  context = clCreateContext(0, 1, &device_id, NULL, NULL, &err);
  if (!context)
  {
    printf("Error: Failed to create a compute context!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }

  // Create a command commands
  //
  commands = clCreateCommandQueue(context, device_id, 0, &err);
  if (!commands)
  {
    printf("Error: Failed to create a command commands!\n");
    printf("Error: code %i\n",err);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }

  int status;

  // Create Program Objects
  //
  
  // Load binary from disk
  unsigned char *kernelbinary;
  char *xclbin=argv[1];
  printf("loading %s\n", xclbin);
  int n_i = load_file_to_memory(xclbin, (char **) &kernelbinary);
  if (n_i < 0) {
    printf("failed to load kernel from xclbin: %s\n", xclbin);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to load kernel from xclbin: %s\n", xclbin);
  }
  size_t n = n_i;
  // Create the compute program from offline
  program = clCreateProgramWithBinary(context, 1, &device_id, &n,
                                      (const unsigned char **) &kernelbinary, &status, &err);
  if ((!program) || (err!=CL_SUCCESS)) {
    printf("Error: Failed to create compute program from binary %d!\n", err);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to create compute program from binary %d!\n", err);
  }

  // Build the program executable
  //
  err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
  if (err != CL_SUCCESS)
  {
    size_t len;
    char buffer[2048];

    printf("Error: Failed to build program executable!\n");
    clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
    printf("%s\n", buffer);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to build program executable!\n");
  }

  // Create the compute kernel in the program we wish to run
  //
  kernel = clCreateKernel(program, "mmult", &err);
  if (!kernel || err != CL_SUCCESS)
  {
    printf("Error: Failed to create compute kernel!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to create compute kernel!\n");
  }

  // Create the input and output arrays in device memory for our calculation
  //
  input_a = clCreateBuffer(context,  CL_MEM_READ_ONLY,  DATA_SIZE*sizeof(int), NULL, NULL);
  output = clCreateBuffer(context, CL_MEM_WRITE_ONLY, RESULT_SIZE*sizeof(int), NULL, NULL);
  if (!input_a || !output)
  {
    printf("Error: Failed to allocate device memory!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }    
  else {
	  printf("Succeed to allocate device memory!\n");
  }

  printf("\n************* Welcome to UCLA FPGA agent! **********\n");
  int listenfd = setupSocket( 5000 );
  int taskNum = -1;
  int totalLen = -1;

  // Get the start time
  timespec timer = tic( );
  timespec socSendTime = diff(timer, timer);
  timespec socRecvTime = diff(timer, timer);
  timespec exeTime = diff(timer, timer);
  while (true) {

    int connfd = acceptSocket(listenfd);
    //printf("\n************* Got a new task! *************\n");
    
    timer = tic();
    totalLen = recv_param(connfd);
    printf ("\nparam received: %d\n", totalLen);
    recv_large_array(connfd, (char*)a, totalLen);
    accTime (&socSendTime, &timer);
    taskNum = a[2];
    printf("\nparameter recieved --- \n");
     //Write our data set into the input array in device memory 
    
    clEnqueueWriteBuffer(commands, input_a, CL_TRUE, 0, totalLen, a, 0, NULL, NULL);
    ////err = clEnqueueWriteBuffer(commands, input_a, CL_TRUE, 0, sizeof(int) * DATA_SIZE, a, 0, NULL, NULL);
    ////if (err != CL_SUCCESS)
    ////{
    ////  printf("Error: Failed to write to source array a!\n");
    ////  printf("Test failed\n");
    ////  return EXIT_FAILURE;
    ////}
      
    // Set the arguments to our compute kernel
    //
    clSetKernelArg(kernel, 0, sizeof(cl_mem), &input_a);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &output);
    clSetKernelArg(kernel, 2, sizeof(int), &taskNum);
    ////err = 0;
    ////err  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &input_a);
    ////err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &output);
    ////err |= clSetKernelArg(kernel, 2, sizeof(int), &taskNum);
    ////if (err != CL_SUCCESS)
    ////{
    ////  printf("Error: Failed to set kernel arguments! %d\n", err);
    ////  printf("Test failed\n");
    ////  return EXIT_FAILURE;
    ////}
  
    // Execute the kernel over the entire range of our 1d input data set
    // using the maximum number of work group items for this device
    //
  
    clEnqueueTask(commands, kernel, 0, NULL, NULL);
    ////err = clEnqueueTask(commands, kernel, 0, NULL, NULL);
    ////if (err)
    ////{
    ////  printf("Error: Failed to execute kernel! %d\n", err);
    ////  printf("Test failed\n");
    ////  return EXIT_FAILURE;
    ////}
  
    // Read back the results from the device to verify the output
    //
    cl_event readevent;
    clEnqueueReadBuffer( commands, output, CL_TRUE, 0, 16*taskNum, results, 0, NULL, &readevent );  
    ////err = clEnqueueReadBuffer( commands, output, CL_TRUE, 0, sizeof(int) * RESULT_SIZE, results, 0, NULL, &readevent );  
    ////if (err != CL_SUCCESS)
    ////{
    ////  printf("Error: Failed to read output array! %d\n", err);
    ////  printf("Test failed\n");
    ////  return EXIT_FAILURE;
    ////}
  
    clWaitForEvents(1, &readevent);
    accTime(&exeTime, &timer);
  
    // Get the execution time
    //toc(&timer);

    send_large_array(connfd, (char*)results, 16*taskNum);
    accTime(&socRecvTime, &timer);
    printf("\n************* Task finished! *************\n");

    close(connfd);
    printf("\n**********timing begin**********\n");
    printTimeSpec(socSendTime);
    printTimeSpec(socRecvTime);
    printTimeSpec(exeTime);
    printf("\n**********timing end**********\n");
  }
    
  // Shutdown and cleanup
  //
  clReleaseMemObject(input_a);
  clReleaseMemObject(output);
  clReleaseProgram(program);
  clReleaseKernel(kernel);
  clReleaseCommandQueue(commands);
  clReleaseContext(context);

  return EXIT_SUCCESS;

}
