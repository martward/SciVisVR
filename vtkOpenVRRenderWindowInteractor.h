/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOpenVRRenderWindowInteractor.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkOpenVRRenderWindowInteractor - implements OpenVR specific functions
// required by vtkRenderWindowInteractor.
//
//
#ifndef vtkOpenVRRenderWindowInteractor_h
#define vtkOpenVRRenderWindowInteractor_h

#include "vtkOpenVRModule.h" // For export macro
#include "vtkRenderWindowInteractor.h"

#include "vtkOpenVRRenderWindow.h" // ivars
#include "vtkNew.h" // ivars
#include "vtkTransform.h" // ivars
#include "Animation.h"

class VTKOPENVR_EXPORT vtkOpenVRRenderWindowInteractor : public vtkRenderWindowInteractor
{
public:
  // Description:
  // Construct object so that light follows camera motion.
  static vtkOpenVRRenderWindowInteractor *New();

  vtkTypeMacro(vtkOpenVRRenderWindowInteractor,vtkRenderWindowInteractor);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Initialize the event handler
  virtual void Initialize();

  // Description:
  // Enable/Disable interactions.  By default interactors are enabled when
  // initialized.  Initialize() must be called prior to enabling/disabling
  // interaction. These methods are used when a window/widget is being
  // shared by multiple renderers and interactors.  This allows a "modal"
  // display where one interactor is active when its data is to be displayed
  // and all other interactors associated with the widget are disabled
  // when their data is not displayed.
  virtual void Enable();
  virtual void Disable();

  virtual void Start();
  int getControllerPort();
  void vtkOpenVRRenderWindowInteractor::processInput(int port);
  double center[3];

  // Description:
  // OpenVR specific application terminate, calls ClassExitMethod then
  // calls PostQuitMessage(0) to terminate the application. An application can Specify
  // ExitMethod for alternative behavior (i.e. suppression of keyboard exit)
  void TerminateApp(void);

  void SetAnimation(AnimationTicker &animation);

  // Description:
  // Methods to set the default exit method for the class. This method is
  // only used if no instance level ExitMethod has been defined.  It is
  // provided as a means to control how an interactor is exited given
  // the various language bindings (tcl, Win32, etc.).
  static void SetClassExitMethod(void (*f)(void *), void *arg);
  static void SetClassExitMethodArgDelete(void (*f)(void *));

  // Description:
  // These methods correspond to the the Exit, User and Pick
  // callbacks. They allow for the Style to invoke them.
  virtual void ExitCallback();

  // Description:
  // Create default picker. Used to create one when none is specified.
  // Default is an instance of vtkPropPicker.
  virtual vtkAbstractPropPicker *CreateDefaultPicker();

  // Description:
  // With OpenVR we know the world coordinate positions
  // and orientations of events. These methods
  // support querying them instead of goign through
  // a display X,Y coordinate approach as is standard
  // for mouse/touch events
  virtual double *GetWorldEventPosition(int pointerIndex)
    {
    if (pointerIndex >= VTKI_MAX_POINTERS)
      {
      return NULL;
      }
    return this->WorldEventPositions[pointerIndex];
    }
  virtual double *GetLastWorldEventPosition(int pointerIndex)
    {
    if (pointerIndex >= VTKI_MAX_POINTERS)
      {
      return NULL;
      }
    return this->LastWorldEventPositions[pointerIndex];
    }
  virtual double *GetWorldEventOrientation(int pointerIndex)
    {
    if (pointerIndex >= VTKI_MAX_POINTERS)
      {
      return NULL;
      }
    return this->WorldEventOrientations[pointerIndex];
    }
  virtual double *GetLastWorldEventOrientation(int pointerIndex)
    {
    if (pointerIndex >= VTKI_MAX_POINTERS)
      {
      return NULL;
      }
    return this->LastWorldEventOrientations[pointerIndex];
    }

  // Description:
  // With OpenVR we know the world coordinate positions
  // and orientations of events. These methods
  // support setting them.
  virtual void SetWorldEventPosition(double x, double y, double z, int pointerIndex)
  {
    if (pointerIndex < 0 || pointerIndex >= VTKI_MAX_POINTERS)
      {
      return;
      }
    vtkDebugMacro(
      << this->GetClassName() << " (" << this
      << "): setting WorldEventPosition to ("
      << x << "," << y << "," << z
      << ") for pointerIndex number " << pointerIndex);
    if (this->WorldEventPositions[pointerIndex][0] != x ||
        this->WorldEventPositions[pointerIndex][1] != y ||
        this->WorldEventPositions[pointerIndex][2] != z ||
        this->LastWorldEventPositions[pointerIndex][0] != x ||
        this->LastWorldEventPositions[pointerIndex][1] != y ||
        this->LastWorldEventPositions[pointerIndex][2] != z)
      {
      this->LastWorldEventPositions[pointerIndex][0] = this->WorldEventPositions[pointerIndex][0];
      this->LastWorldEventPositions[pointerIndex][1] = this->WorldEventPositions[pointerIndex][1];
      this->LastWorldEventPositions[pointerIndex][2] = this->WorldEventPositions[pointerIndex][2];
      this->WorldEventPositions[pointerIndex][0] = x;
      this->WorldEventPositions[pointerIndex][1] = y;
      this->WorldEventPositions[pointerIndex][2] = z;
      this->Modified();
      }
  }
  virtual void SetWorldEventOrientation(double w, double x, double y, double z, int pointerIndex)
  {
    if (pointerIndex < 0 || pointerIndex >= VTKI_MAX_POINTERS)
      {
      return;
      }
    vtkDebugMacro(
      << this->GetClassName() << " (" << this
      << "): setting WorldEventOrientation to ("
      << w << "," << x << "," << y << "," << z
      << ") for pointerIndex number " << pointerIndex);
    if (this->WorldEventOrientations[pointerIndex][0] != w ||
        this->WorldEventOrientations[pointerIndex][1] != x ||
        this->WorldEventOrientations[pointerIndex][2] != y ||
        this->WorldEventOrientations[pointerIndex][3] != z ||
        this->LastWorldEventOrientations[pointerIndex][0] != w ||
        this->LastWorldEventOrientations[pointerIndex][1] != x ||
        this->LastWorldEventOrientations[pointerIndex][2] != y ||
        this->LastWorldEventOrientations[pointerIndex][3] != z)
      {
      this->LastWorldEventOrientations[pointerIndex][0] = this->WorldEventOrientations[pointerIndex][0];
      this->LastWorldEventOrientations[pointerIndex][1] = this->WorldEventOrientations[pointerIndex][1];
      this->LastWorldEventOrientations[pointerIndex][2] = this->WorldEventOrientations[pointerIndex][2];
      this->LastWorldEventOrientations[pointerIndex][3] = this->WorldEventOrientations[pointerIndex][3];
      this->WorldEventOrientations[pointerIndex][0] = w;
      this->WorldEventOrientations[pointerIndex][1] = x;
      this->WorldEventOrientations[pointerIndex][2] = y;
      this->WorldEventOrientations[pointerIndex][3] = z;
      this->Modified();
      }
  }

  // Description:
  // Override to set pointers down
  virtual void RightButtonPressEvent();
  virtual void RightButtonReleaseEvent();

  // Description:
  // Set/Get the latest trackpad position
  vtkSetVector2Macro(TouchPadPosition,float);
  vtkGetVector2Macro(TouchPadPosition,float);

protected:
  vtkOpenVRRenderWindowInteractor();
  ~vtkOpenVRRenderWindowInteractor();

  int     MouseInWindow;
  int     StartedMessageLoop;
  float TouchPadPosition[2];
  void UpdateTouchPadPosition(vr::IVRSystem *pHMD,
     vr::TrackedDeviceIndex_t tdi);

  bool Done;  // is the event loop done running

  // Description:
  // Class variables so an exit method can be defined for this class
  // (used to set different exit methods for various language bindings,
  // i.e. tcl, java, Win32)
  static void (*ClassExitMethod)(void *);
  static void (*ClassExitMethodArgDelete)(void *);
  static void *ClassExitMethodArg;

  // Description:
  // Win32-specific internal timer methods. See the superclass for detailed
  // documentation.
  virtual int InternalCreateTimer(int timerId, int timerType, unsigned long duration);
  virtual int InternalDestroyTimer(int platformTimerId);

  // Description:
  // This will start up the event loop and never return. If you
  // call this method it will loop processing events until the
  // application is exited.
  virtual void StartEventLoop();
  //virtual void MartijnSebasEventLoop(AnimationTicker &animation);

  vtkNew<vtkTransform> PoseTransform;

  // converts a device pose to a world coordinate
  // position and orientation
  void ConvertPoseToWorldCoordinates(
    vtkRenderer *ren,
    vr::TrackedDevicePose_t &tdPose,
    double pos[3],
    double wxyz[4]);

  double   WorldEventPositions[VTKI_MAX_POINTERS][3];
  double   LastWorldEventPositions[VTKI_MAX_POINTERS][3];
  double   WorldEventOrientations[VTKI_MAX_POINTERS][4];
  double   LastWorldEventOrientations[VTKI_MAX_POINTERS][4];

private:
  vtkOpenVRRenderWindowInteractor(const vtkOpenVRRenderWindowInteractor&);  // Not implemented.
  void operator=(const vtkOpenVRRenderWindowInteractor&);  // Not implemented.
  AnimationTicker *animation;
};

#endif
