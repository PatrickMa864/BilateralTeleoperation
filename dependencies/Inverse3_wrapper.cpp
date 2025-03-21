
/*
 * Include Files
 *
 */
#include "simstruc.h"



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include "hardwareAPI.h"
#include "mex.h"
#include <typeinfo>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 3
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
using namespace Haply::HardwareAPI::Devices;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void Inverse3_Start_wrapper(void **pW,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
// Set device port name (find it in your computer's device manager)
    char *portName = "COM7";

    // Initalize persistant varaibles 
    pW[0] = (void *) new bool;
    pW[2] = (void *) new Haply::HardwareAPI::IO::SerialStream(portName);
    pW[1] = (void *) new Inverse3((Haply::HardwareAPI::IO::SerialStream *) pW[2]);
    
    // Assign the pointers to varaibles that need to be used in this function 
    Inverse3 * device = (Inverse3 *) pW[1];  
    bool inverse3Found = (bool *) pW[0];

    // Wakup the device 
    Inverse3::DeviceInfoResponse deviceInfo = device->DeviceWakeup();

    // If the device output is valid 
    // Print device info and save the information
    if (deviceInfo.device_id != 0){

        mexPrintf("Inverse3 found\n");
        mexPrintf("\tid: %i\n",deviceInfo.device_id);
        mexPrintf("\tmodel: %i\n",deviceInfo.device_model_number);
        mexPrintf("\thardware version: %i\n",deviceInfo.hardware_version);
        inverse3Found = true;
        
    }
    else{
        mexPrintf("Inverse3 not found\n");
        inverse3Found = false;
    }
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void Inverse3_Outputs_wrapper(const real32_T *force,
			real32_T *position,
			real32_T *velocity,
			void **pW,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
// Copy the pointers to objects needed to execute the function
    bool * inverse3Found = (bool *) pW[0];
    Inverse3 * device = (Inverse3 *) pW[1];   
    

    Inverse3::EndEffectorStateResponse ee_response;
    
    if (inverse3Found){
        // Send desired force
        ee_response = device->EndEffectorForce({force[0],force[1],force[2]});
        
        // Assign the outputs to the relevent vectors
        for (int cc = 0; cc < 3; cc++){
            position[cc] = (real32_T)ee_response.position[cc];
            velocity[cc] = (real32_T)ee_response.velocity[cc];
        }

        // Uncomment these to see position and velocity in the message window
//       mexPrintf("position:\n\tx  %f\n\ty  %f\n\tz  %f\n",ee_response.position[0],ee_response.position[1],ee_response.position[2]);
//       mexPrintf("velocity:\n\tx  %f\n\ty  %f\n\tz  %f\n",ee_response.velocity[0],ee_response.velocity[1],ee_response.velocity[2]);
    }
    // If device was not found, output zeros
    else{
        for (int cc = 0; cc < 3; cc++){
            position[cc] = 0;
            velocity[cc] = 0;
        }
    }
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
void Inverse3_Terminate_wrapper(void **pW,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
// Copy the pointers to objects needed to execute the function
    bool * inverse3Found = (bool *) pW[0];
    Inverse3 * device = (Inverse3 *) pW[1];  
    Haply::HardwareAPI::IO::SerialStream * inverseStream = (Haply::HardwareAPI::IO::SerialStream *) pW[2];

    // Close the serial communication with the device 
    if (inverse3Found){
        inverseStream->CloseDevice();
    }
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

