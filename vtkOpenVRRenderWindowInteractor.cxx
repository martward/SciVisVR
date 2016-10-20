/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOpenVRRenderWindowInteractor.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cassert>

#include "vtkOpenVRRenderWindowInteractor.h"
#include "vtkOpenVRRenderWindow.h"
#include "vtkRendererCollection.h"


#include "vtkActor.h"
#include "vtkObjectFactory.h"
#include "vtkCommand.h"
#include "vtkOpenVRCamera.h"
#include "vtkNew.h"
#include "vtkInteractorStyleOpenVR.h"
#include "vtkOpenVRPropPicker.h"

#include <Windows.h>
#include <Xinput.h>

#include <iostream>


vtkStandardNewMacro(vtkOpenVRRenderWindowInteractor);

void (*vtkOpenVRRenderWindowInteractor::ClassExitMethod)(void *) = (void (*)(void *))NULL;
void *vtkOpenVRRenderWindowInteractor::ClassExitMethodArg = (void *)NULL;
void (*vtkOpenVRRenderWindowInteractor::ClassExitMethodArgDelete)(void *) = (void (*)(void *))NULL;

//----------------------------------------------------------------------------
// Construct object so that light follows camera motion.
vtkOpenVRRenderWindowInteractor::vtkOpenVRRenderWindowInteractor()
{
  this->MouseInWindow = 0;
  this->StartedMessageLoop = 0;
  vtkNew<vtkInteractorStyleOpenVR> style;
  this->SetInteractorStyle(style.Get());
}

//----------------------------------------------------------------------------
vtkOpenVRRenderWindowInteractor::~vtkOpenVRRenderWindowInteractor()
{
}

//----------------------------------------------------------------------
// Creates an instance of vtkPropPicker by default
vtkAbstractPropPicker *vtkOpenVRRenderWindowInteractor::CreateDefaultPicker()
{
  return vtkOpenVRPropPicker::New();
}

void vtkOpenVRRenderWindowInteractor::ConvertPoseToWorldCoordinates(
  vtkRenderer *ren,
  vr::TrackedDevicePose_t &tdPose,
  double pos[3],
  double wxyz[4])
{
  // get the position and orientation of the button press
  for (int i = 0; i < 3; i++)
    {
    pos[i] = tdPose.mDeviceToAbsoluteTracking.m[i][3];
    }

  vtkOpenVRCamera *cam =
    static_cast<vtkOpenVRCamera *>(ren->GetActiveCamera());
  double scale = cam->GetScale();
  double *trans = cam->GetTranslation();

  for (int i = 0; i < 3; i++)
    {
    pos[i] = pos[i]/scale - trans[i];
    }

  double ortho[3][3];
  for (int i = 0; i < 3; i++)
    {
    ortho[0][i] = tdPose.mDeviceToAbsoluteTracking.m[0][i];
    ortho[1][i] = tdPose.mDeviceToAbsoluteTracking.m[1][i];
    ortho[2][i] = tdPose.mDeviceToAbsoluteTracking.m[2][i];
    }
  if (vtkMath::Determinant3x3(ortho) < 0)
    {
    ortho[0][2] = -ortho[0][2];
    ortho[1][2] = -ortho[1][2];
    ortho[2][2] = -ortho[2][2];
    }
  vtkMath::Matrix3x3ToQuaternion(ortho, wxyz);

  // calc the return value wxyz
 double mag = sqrt( wxyz[1] * wxyz[1] + wxyz[2] * wxyz[2] + wxyz[3] * wxyz[3] );

  if ( mag != 0.0 )
    {
    wxyz[0] = 2.0 * vtkMath::DegreesFromRadians( atan2( mag, wxyz[0] ) );
    wxyz[1] /= mag;
    wxyz[2] /= mag;
    wxyz[3] /= mag;
    }
  else
    {
    wxyz[0] = 0.0;
    wxyz[1] = 0.0;
    wxyz[2] = 0.0;
    wxyz[3] = 1.0;
    }
}

void vtkOpenVRRenderWindowInteractor::UpdateTouchPadPosition(
  vr::IVRSystem *pHMD,
   vr::TrackedDeviceIndex_t tdi)
{
  vr::VRControllerState_t cstate;
  pHMD->GetControllerState(tdi,&cstate);

  for (unsigned int i = 0; i < vr::k_unControllerStateAxisCount; i++)
    {
    if (pHMD->GetInt32TrackedDeviceProperty(tdi,
      static_cast<vr::ETrackedDeviceProperty>(vr::ETrackedDeviceProperty::Prop_Axis0Type_Int32 + i))
      == vr::EVRControllerAxisType::k_eControllerAxis_TrackPad)
      {
      this->SetTouchPadPosition(cstate.rAxis[i].x,cstate.rAxis[i].y);
      }
    }
}

void vtkOpenVRRenderWindowInteractor::Start()
{
	std::cout << "start" << std::endl;
	// Let the compositing handle the event loop if it wants to.
	if (this->HasObserver(vtkCommand::StartEvent) && !this->HandleEventLoop)
	{
		this->InvokeEvent(vtkCommand::StartEvent, NULL);
		return;
	}

	// As a convenience, initialize if we aren't initialized yet.
	if (!this->Initialized)
	{
		this->Initialize();

		if (!this->Initialized)
		{
			return;
		}
	}

	// Pass execution to the subclass which will run the event loop,
	// this will not return until TerminateApp is called.
	this->StartEventLoop();
}

int vtkOpenVRRenderWindowInteractor::getControllerPort()
{
	XINPUT_STATE state;
	int port = -1;
	for (int i = 0; i < XUSER_MAX_COUNT; i++)
	{
		if (XInputGetState(i, &state) == ERROR_SUCCESS)
		{
			std::cout << "found controller on port " << i << std::endl;
			return i;
		}
	}
}

void vtkOpenVRRenderWindowInteractor::processInput(int port)
{
	bool lLeft, lRight, lUp, lDown, lTrigger, lButton;
	bool rLeft, rRight, rUp, rDown, rTrigger, rButton;
	XINPUT_STATE state;
	if (XInputGetState(port, &state) == ERROR_SUCCESS)
	{
		if ( state.Gamepad.wButtons & XINPUT_GAMEPAD_X > 0)
		{
			center[0] = 0;
			center[1] = 0;
			center[2] = 0;
		}

		lLeft = -1 == fmaxf(-1, state.Gamepad.sThumbLX / 32767);
		lRight = 1 == fmaxf(-1, state.Gamepad.sThumbLX / 32767);

		lUp = 1 == fmaxf(-1, state.Gamepad.sThumbLY / 32767);
		lDown = -1 == fmaxf(-1, state.Gamepad.sThumbLY / 32767);

		rLeft = -1 == fmaxf(-1, state.Gamepad.sThumbRX / 32767);
		rRight = 1 == fmaxf(-1, state.Gamepad.sThumbRX / 32767);

		rUp = 1 == fmaxf(-1, state.Gamepad.sThumbRY / 32767);
		rDown = -1 == fmaxf(-1, state.Gamepad.sThumbRY / 32767);

		lTrigger = 1 == ceil(state.Gamepad.bLeftTrigger / 255);
		lButton = 1 == (state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) / 256;

		rTrigger = 1 == ceil(state.Gamepad.bRightTrigger / 255);
		rButton = 1 == (state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) / 512;

		if (lLeft || lRight || lUp || lDown || lButton || lTrigger)
		{
			double stepSize = 1;
			//double *center = animation->source1->GetCenter();

			for (int i = 0; i < 3; i++)
			{
				std::cout << center[i] << ", ";
			}
			std::cout << std::endl;

			if (lLeft) center[0] -= stepSize;
			if (lRight) center[0] += stepSize;

			if (lUp) center[1] += stepSize;
			if (lDown) center[1] -= stepSize;

			if (lButton) center[2] += stepSize;
			if (lTrigger) center[2] -= stepSize;

			if (lLeft || lRight || lUp || lDown || lButton || lTrigger)
				std::cout << "A button happenend" << std::endl;

			for (int i = 0; i < 3; i++)
			{
				std::cout << center[i] << ", ";
			}
			std::cout << std::endl;
			std::cout << "---------------" << std::endl;
			
			animation->source1->Update();
		}

		animation->source1->SetCenter(center);

	}
	else
	{
		std::cout << "COULD NOT READ FROM CONTROLLER" << std::endl;
	}
}

//----------------------------------------------------------------------------
void  vtkOpenVRRenderWindowInteractor::StartEventLoop()
{
  this->StartedMessageLoop = 1;
  this->Done = false;

  vtkOpenVRRenderWindow *renWin =
    vtkOpenVRRenderWindow::SafeDownCast(this->RenderWindow);

  vr::IVRSystem *pHMD = renWin->GetHMD();

  vr::VREvent_t event;
//  vr::TrackedDevicePose_t tdPose;

  vtkCollectionSimpleIterator rit;
  renWin->GetRenderers()->InitTraversal(rit);
  vtkRenderer *ren = renWin->GetRenderers()->GetNextRenderer(rit);

  int iterators[200];

  double iteration = 0;

  int controllerPort = 0;
  if (animation)
  {
	  controllerPort = getControllerPort();
  }

  while (!this->Done)
  {
	  if (animation)
	  {
		  if (animation->source1)
		  {
			  /*
			  double radius = 10;

			  double x = sin(2.0 * M_PI * iteration / 100)*radius + animation->centerX;
			  double y = cos(2.0 * M_PI * iteration / 100)*radius + animation->centerY;
			  std::cout << "cirlce: " << x << ', ' << y << std::endl;
			  animation->source1->SetCenter(0, x, y);

			  iteration = (iteration + 0.1);
			  if (iteration > 100) iteration = 0;
			  */

			  processInput(controllerPort);
			  
			  int actorIndex = 0;
			  for (int j = 0; j < animation->tracers.size(); j++)
			  {
				  vtkSmartPointer<vtkStreamTracer> tracer = animation->tracers.at(j);
				  tracer->GetOutput()->GetLines()->InitTraversal();
				  //std::cout << "tracer: " << j << std::endl;
				  for (int i = 0; i < tracer->GetOutput()->GetNumberOfLines(); i++)
				  {
					  vtkSmartPointer<vtkActor> actor = animation->actors.at(actorIndex);
					  vtkIdType *pts;
					  vtkIdType npts;
					  tracer->GetOutput()->GetLines()->GetNextCell(npts, pts);
					  //std::cout << *pts << std::endl;

					  actor->SetPosition(tracer->GetOutput()->GetPoint((int)*pts + iterators[actorIndex] % (int)npts));

					  iterators[actorIndex] = (iterators[actorIndex] + 3) % (int)npts;
					  actorIndex++;
				  }

			  }
		  }
	  }
    bool result =
      // pHMD->PollNextEventWithPose(
      //   vr::VRCompositor()->GetTrackingSpace(),
      //   &event,
      //   sizeof(vr::VREvent_t),
      //   &tdPose );
      pHMD->PollNextEvent(
        &event,
        sizeof(vr::VREvent_t));

    if (result)
      {
      vr::TrackedDeviceIndex_t tdi = event.trackedDeviceIndex;
      // is it a controller button action?
      if (pHMD->GetTrackedDeviceClass(tdi) ==
          vr::ETrackedDeviceClass::TrackedDeviceClass_Controller &&
            (event.eventType == vr::VREvent_ButtonPress ||
             event.eventType == vr::VREvent_ButtonUnpress))
        {
        vr::ETrackedControllerRole role = pHMD->GetControllerRoleForTrackedDeviceIndex(tdi);

        this->UpdateTouchPadPosition(pHMD,tdi);

        // 0 = right hand 1 = left
        int pointerIndex =
          (role == vr::ETrackedControllerRole::TrackedControllerRole_RightHand ? 0 : 1);
        this->PointerIndexLookup[pointerIndex] = tdi;

        vr::TrackedDevicePose_t &tdPose = renWin->GetTrackedDevicePose(tdi);
        double pos[3];
        double wxyz[4];
        this->ConvertPoseToWorldCoordinates(ren, tdPose, pos, wxyz);

        // so even though we have world coordinates we have to convert them to
        // screen coordinates because all of VTKs picking code is currently
        // based on screen coordinates
        ren->SetWorldPoint(pos[0],pos[1],pos[2],1.0);
        ren->WorldToDisplay();
        double *displayCoords = ren->GetDisplayPoint();

        this->SetEventPosition(displayCoords[0],displayCoords[1],pointerIndex);
        this->SetWorldEventPosition(pos[0],pos[1],pos[2],pointerIndex);
        this->SetWorldEventOrientation(wxyz[0],wxyz[1],wxyz[2],wxyz[3],pointerIndex);
        this->SetPointerIndex(pointerIndex);

        if (event.eventType == vr::VREvent_ButtonPress)
          {
          if (event.data.controller.button == vr::EVRButtonId::k_EButton_Axis1)
            {
            this->LeftButtonPressEvent();
            }
          if (event.data.controller.button == vr::EVRButtonId::k_EButton_Axis0)
            {
            this->RightButtonPressEvent();
            }
          if (event.data.controller.button == vr::EVRButtonId::k_EButton_ApplicationMenu)
            {
            this->Done = true;
            }
          }
        if (event.eventType == vr::VREvent_ButtonUnpress)
          {
          if (event.data.controller.button == vr::EVRButtonId::k_EButton_Axis1)
            {
            this->LeftButtonReleaseEvent();
            }
          if (event.data.controller.button == vr::EVRButtonId::k_EButton_Axis0)
            {
            this->RightButtonReleaseEvent();
            }
          }
        }
      }
    else
      {
      // if pointers are down track the movement
      if (this->PointersDownCount)
        {
        for (int i = 0; i < VTKI_MAX_POINTERS; i++)
          {
          if (this->PointersDown[i])
            {
            this->UpdateTouchPadPosition(pHMD,this->PointerIndexLookup[i]);
            vr::TrackedDevicePose_t &tdPose =
              renWin->GetTrackedDevicePose(
              static_cast<vr::TrackedDeviceIndex_t>(this->PointerIndexLookup[i]));
            double pos[3];
            double wxyz[4];
            this->ConvertPoseToWorldCoordinates(ren, tdPose, pos, wxyz);

            // so even though we have world coordinates we have to convert them to
            // screen coordinates because all of VTKs picking code is currently
            // based on screen coordinates
            ren->SetWorldPoint(pos[0],pos[1],pos[2],1.0);
            ren->WorldToDisplay();
            double *displayCoords = ren->GetDisplayPoint();
            this->SetEventPosition(displayCoords[0], displayCoords[1], i);
            this->SetWorldEventPosition(pos[0],pos[1],pos[2],i);
            this->SetWorldEventOrientation(wxyz[0],wxyz[1],wxyz[2],wxyz[3],i);
            }
          }
        this->MouseMoveEvent();
        }
      }

    renWin->Render();
    }
}

void vtkOpenVRRenderWindowInteractor::SetAnimation(AnimationTicker &animation)
{
	this->animation = &animation;
}

//----------------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::Initialize()
{
  // make sure we have a RenderWindow and camera
  if ( ! this->RenderWindow)
    {
    vtkErrorMacro(<<"No renderer defined!");
    return;
    }
  if (this->Initialized)
    {
    return;
    }

  vtkOpenVRRenderWindow *ren =
    vtkOpenVRRenderWindow::SafeDownCast(this->RenderWindow);
  int *size;

  this->Initialized = 1;
  // get the info we need from the RenderingWindow

  size = ren->GetSize();
  ren->GetPosition();
  this->Enable();
  this->Size[0] = size[0];
  this->Size[1] = size[1];
}

//----------------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::Enable()
{
  if (this->Enabled)
    {
    return;
    }
  this->Enabled = 1;
  this->Modified();
}


//----------------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::Disable()
{
  if (!this->Enabled)
    {
    return;
    }

  this->Enabled = 0;
  this->Modified();
}

//----------------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::TerminateApp(void)
{
  this->Done = true;
}

//----------------------------------------------------------------------------
int vtkOpenVRRenderWindowInteractor::InternalCreateTimer(int timerId, int vtkNotUsed(timerType),
                                                          unsigned long duration)
{
  // todo
  return 0;
}

//----------------------------------------------------------------------------
int vtkOpenVRRenderWindowInteractor::InternalDestroyTimer(int platformTimerId)
{
  // todo
  return 0;
}


//----------------------------------------------------------------------------
// Specify the default function to be called when an interactor needs to exit.
// This callback is overridden by an instance ExitMethod that is defined.
void
vtkOpenVRRenderWindowInteractor::SetClassExitMethod(void (*f)(void *),void *arg)
{
  if ( f != vtkOpenVRRenderWindowInteractor::ClassExitMethod
       || arg != vtkOpenVRRenderWindowInteractor::ClassExitMethodArg)
    {
    // delete the current arg if there is a delete method
    if ((vtkOpenVRRenderWindowInteractor::ClassExitMethodArg)
        && (vtkOpenVRRenderWindowInteractor::ClassExitMethodArgDelete))
      {
      (*vtkOpenVRRenderWindowInteractor::ClassExitMethodArgDelete)
        (vtkOpenVRRenderWindowInteractor::ClassExitMethodArg);
      }
    vtkOpenVRRenderWindowInteractor::ClassExitMethod = f;
    vtkOpenVRRenderWindowInteractor::ClassExitMethodArg = arg;

    // no call to this->Modified() since this is a class member function
    }
}

//----------------------------------------------------------------------------
// Set the arg delete method.  This is used to free user memory.
void
vtkOpenVRRenderWindowInteractor::SetClassExitMethodArgDelete(void (*f)(void *))
{
  if (f != vtkOpenVRRenderWindowInteractor::ClassExitMethodArgDelete)
    {
    vtkOpenVRRenderWindowInteractor::ClassExitMethodArgDelete = f;

    // no call to this->Modified() since this is a class member function
    }
}

//----------------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  os << indent << "StartedMessageLoop: " << this->StartedMessageLoop << endl;
}

//----------------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::ExitCallback()
{
  if (this->HasObserver(vtkCommand::ExitEvent))
    {
    this->InvokeEvent(vtkCommand::ExitEvent,NULL);
    }
  else if (this->ClassExitMethod)
    {
    (*this->ClassExitMethod)(this->ClassExitMethodArg);
    }

  this->TerminateApp();
}

//------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::RightButtonPressEvent()
{
  if (!this->Enabled)
    {
    return;
    }

  // are we translating multitouch into gestures?
  if (this->RecognizeGestures)
    {
    if (!this->PointersDown[this->PointerIndex])
      {
      this->PointersDown[this->PointerIndex] = 1;
      this->PointersDownCount++;
      }
    // do we have multitouch
    if (this->PointersDownCount > 1)
      {
      // did we just transition to multitouch?
      if (this->PointersDownCount == 2)
        {
        this->InvokeEvent(vtkCommand::RightButtonReleaseEvent, NULL);
        }
      // handle the gesture
      this->RecognizeGesture(vtkCommand::RightButtonPressEvent);
      return;
      }
    }

  this->InvokeEvent(vtkCommand::RightButtonPressEvent, NULL);
}

//------------------------------------------------------------------
void vtkOpenVRRenderWindowInteractor::RightButtonReleaseEvent()
{
  if (!this->Enabled)
    {
    return;
    }

  if (this->RecognizeGestures)
    {
    if (this->PointersDown[this->PointerIndex])
      {
      this->PointersDown[this->PointerIndex] = 0;
      this->PointersDownCount--;
      }
    // do we have multitouch
    if (this->PointersDownCount > 1)
      {
      // handle the gesture
      this->RecognizeGesture(vtkCommand::RightButtonReleaseEvent);
      return;
      }
    }
  this->InvokeEvent(vtkCommand::RightButtonReleaseEvent, NULL);
}
