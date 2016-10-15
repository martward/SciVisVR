/*=========================================================================

  Program:   Visualization Toolkit

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkCamera.h"
#include "vtkRenderer.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkActor.h"
#include "vtkPolyDataMapper.h"
#include "vtkPLYReader.h"
#include "vtkNew.h"
#include "vtkProperty.h"

#include "vtkRegressionTestImage.h"
#include "vtkTestUtilities.h"

#include "vtkRenderWindowInteractor.h"

#include "vtkOpenGLRenderWindow.h"

#include "vtkOpenVRCamera.h"
#include "vtkCullerCollection.h"
#include "vtkTransform.h"

#include "vtkPlaneWidget.h"

#include "vtkTransformPolyDataFilter.h"

#include "vtkLight.h"

#include <vtkVersion.h>
#include <vtkStructuredPointsReader.h>
#include <vtkStructuredPoints.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStreamTracer.h>
#include <vtkStructuredGridReader.h>
#include <vtkPointSource.h>
#include <vtkRungeKutta2.h>
#include <vtkTubeFilter.h>
#include <vtkOutlineFilter.h>
#include <vtkContourFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <Windows.h>
#include <Xinput.h>

#include <thread>         // std::thread
#include <chrono>

std::string fileName;

class ControllerProxy {
public:
	ControllerProxy(vtkSmartPointer<vtkPointSource> source1, vtkSmartPointer<vtkPointSource> source2, vtkSmartPointer<vtkRenderWindow> renderWindow);
	int getControllerPort();
	XINPUT_STATE state;
	void run();
	void setPointSources(vtkSmartPointer<vtkPointSource> source1, vtkSmartPointer<vtkPointSource> source2);

private:
	int port;
	void processInput();
	bool lLeft, lRight, lUp, lDown, lTrigger, lButton;
	bool rLeft, rRight, rUp, rDown, rTrigger, rButton;

	vtkSmartPointer<vtkPointSource> source1, source2;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
};

int ControllerProxy::getControllerPort()
{
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

void ControllerProxy::processInput()
{
	if (XInputGetState(port, &state) == ERROR_SUCCESS)
	{
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
			double stepSize = 10;
			double *center = source1->GetCenter();

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
			source1->SetCenter(center);

			std::cout << "renderwindow2: " << renderWindow->GetNeverRendered() << std::endl;
			if (renderWindow)
			{
				std::cout << "render" << std::endl;
				source1->Update();
				renderWindow->Render();
			}
				
		}	
	}
	else
	{
		std::cout << "COULD NOT READ FROM CONTROLLER" << std::endl;
	}
}

ControllerProxy::ControllerProxy(vtkSmartPointer<vtkPointSource> source1, vtkSmartPointer<vtkPointSource> source2, vtkSmartPointer<vtkRenderWindow> renderWindow)
{
	this->source1 = source1;
	this->source2 = source2;
	this->renderWindow = renderWindow;
	port = getControllerPort();
}

void ControllerProxy::run()
{
	while (true)
	{
		processInput();
		std::this_thread::sleep_for(std::chrono::milliseconds(1000/90));
	}
}

class vtkCallBack : public vtkCommand {
public:
	static vtkCallBack *New()
	{
		return new vtkCallBack;
	}

	virtual void Execute(vtkObject *caller, unsigned long eventId, void* cellData)
	{
		std::cout << "Event: " << vtkCommand::GetStringFromEventId(eventId) << std::endl;
		if (eventId == vtkCommand::TimerEvent)
		{
			std::cout << "animation step " << iteration << std::endl;
			double radius = 10;

			x = sin(2.0 * 3.14 * iteration / 100)*radius + centerX;
			y = cos(2.0 * 3.14 * iteration / 100)*radius + centerY;

			pointSource->SetCenter(0, x, y);

			iteration = (iteration + 1) % 100;

			vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
			iren->GetRenderWindow()->Render();
		}
	}

	void setup(vtkSmartPointer<vtkPointSource> pointSource1, double x1, double y1)
	{
		this->pointSource = pointSource1;
		centerX = x1*0.5;
		centerY = y1*0.5;

		iteration = 0;
	}

private:
	vtkSmartPointer<vtkPointSource> pointSource;
	double x, y;
	double centerX, centerY;
	int iteration;
};



class FlowVisualiser
{
private:
	const static int nPoints = 100;
	const static int maxPropagation = 150;
	const double red[3];
	const double green[3];
	const double blue[3];
	const double r = 15;

public:

	FlowVisualiser(std::string fileName, vtkSmartPointer<vtkPointSource> source3, vtkSmartPointer<vtkPointSource> source4, vtkSmartPointer<vtkRenderWindow> renderWindow);
	vtkSmartPointer<vtkActor> getStreamTraceActor(const double color[],
		vtkAlgorithmOutput *flowFieldInput,
		vtkSmartPointer<vtkPointSource> pointSource,
		vtkSmartPointer<vtkStreamTracer> streamTracer);

	vtkSmartPointer<vtkPointSource> getPointSource(double x, double y, double z, double r, int nPoints);
	vtkSmartPointer<vtkPointSource> setPointSource(vtkSmartPointer<vtkPointSource> pointSource, double x, double y, double z, double r, int nPoints);
};

vtkSmartPointer<vtkPointSource> FlowVisualiser::setPointSource(vtkSmartPointer<vtkPointSource> pointSource, double x, double y, double z, double r, int nPoints)
{
	// Generate random points
	pointSource->SetRadius(r);
	pointSource->SetNumberOfPoints(nPoints);
	pointSource->SetCenter(x, y, z);
	pointSource->Update();

	return pointSource;
}

vtkSmartPointer<vtkPointSource> FlowVisualiser::getPointSource(double x, double y, double z, double r, int nPoints)
{
	// Generate random points
	vtkSmartPointer<vtkPointSource> pointSource = vtkSmartPointer<vtkPointSource>::New();
	pointSource->SetRadius(r);
	pointSource->SetNumberOfPoints(nPoints);
	pointSource->SetCenter(x, y, z);
	pointSource->Update();

	return pointSource;
}

vtkSmartPointer<vtkActor> FlowVisualiser::getStreamTraceActor(const double color[],
	vtkAlgorithmOutput *flowFieldInput,
	vtkSmartPointer<vtkPointSource> pointSource,
	vtkSmartPointer<vtkStreamTracer> streamTracer)
{

	vtkSmartPointer<vtkRungeKutta2> integ = vtkSmartPointer<vtkRungeKutta2>::New();

	// Stream trace
	streamTracer = vtkSmartPointer<vtkStreamTracer>::New();
	streamTracer->SetInputConnection(flowFieldInput);
	streamTracer->SetSourceConnection(pointSource->GetOutputPort());
	streamTracer->SetMaximumPropagation(maxPropagation);
	streamTracer->SetInitialIntegrationStep(0.1);
	streamTracer->SetIntegrationDirectionToBoth();
	streamTracer->SetIntegrator(integ);
	streamTracer->SetComputeVorticity(false);
	streamTracer->SetSurfaceStreamlines(false);

	// Create tubes
	vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
	tubeFilter->SetInputConnection(streamTracer->GetOutputPort());
	tubeFilter->SetRadius(0.1);
	tubeFilter->SetNumberOfSides(50);
	tubeFilter->Update();

	// Mapper and Actor
	vtkSmartPointer<vtkPolyDataMapper> tubeMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	tubeMapper->ScalarVisibilityOff();
	tubeMapper->SetInputConnection(tubeFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> tubeActor =
		vtkSmartPointer<vtkActor>::New();
	tubeActor->SetMapper(tubeMapper);
	tubeActor->GetProperty()->SetColor((double*)color);
	return tubeActor;
}

FlowVisualiser::FlowVisualiser(std::string fileName, vtkSmartPointer<vtkPointSource> source3, vtkSmartPointer<vtkPointSource> source4, vtkSmartPointer<vtkRenderWindow> renderWindow) :
	red{ 255, 0, 0 }, green{ 0, 255, 0 }, blue{ 0, 0, 255 }
{
	// Read flow field
	vtkSmartPointer<vtkStructuredPointsReader> pointsReader = vtkSmartPointer<vtkStructuredPointsReader>::New();
	pointsReader->SetFileName(fileName.c_str());
	pointsReader->Update();

	double bounds[6];
	pointsReader->GetOutput()->GetBounds(bounds);

	// Create the outline
	vtkSmartPointer<vtkOutlineFilter> outline =
		vtkSmartPointer<vtkOutlineFilter>::New();
	outline->SetInputConnection(pointsReader->GetOutputPort());

	// Create contour to find the mixing object
	vtkSmartPointer<vtkContourFilter> contour = vtkSmartPointer<vtkContourFilter>::New();
	contour->SetInputConnection(pointsReader->GetOutputPort());
	contour->SetNumberOfContours(0);
	contour->SetValue(0, 10);

	// Create point sources
	vtkSmartPointer<vtkPointSource> source1 = getPointSource(0, bounds[3] * 0.25, bounds[5] / 2, r, nPoints);
	vtkSmartPointer<vtkPointSource> source2 = getPointSource(0, bounds[3] * 0.75, bounds[5] / 2, r, nPoints);
	source3 = setPointSource(source3, 0, bounds[3] * 0.5, bounds[5] / 2, 0, 1);

	// Create traces
	vtkSmartPointer<vtkStreamTracer> streamTracer1;
	vtkSmartPointer<vtkStreamTracer> streamTracer2;
	vtkSmartPointer<vtkStreamTracer> streamTracer3;

	vtkSmartPointer<vtkActor> trace1 = getStreamTraceActor(red, pointsReader->GetOutputPort(), source1, streamTracer1);
	vtkSmartPointer<vtkActor> trace2 = getStreamTraceActor(green, pointsReader->GetOutputPort(), source2, streamTracer2);
	vtkSmartPointer<vtkActor> trace3 = getStreamTraceActor(blue, pointsReader->GetOutputPort(), source3, streamTracer3);

	// Show source3
	vtkSmartPointer<vtkPolyDataMapper> source3Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	source3Mapper->SetInputConnection(source3->GetOutputPort());
	source3Mapper->ScalarVisibilityOff();
	vtkSmartPointer<vtkActor> source3Actor = vtkSmartPointer<vtkActor>::New();
	source3Actor->SetMapper(source3Mapper);
	source3Actor->GetProperty()->SetColor(0, 1, 1);

	//Map outline
	vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	outlineMapper->SetInputConnection(outline->GetOutputPort());
	vtkSmartPointer<vtkActor> outlineActor =
		vtkSmartPointer<vtkActor>::New();
	outlineActor->SetMapper(outlineMapper);
	outlineActor->GetProperty()->SetColor(1, 1, 1);

	// Map contour
	vtkSmartPointer<vtkPolyDataMapper> contourMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	contourMapper->SetInputConnection(contour->GetOutputPort());
	contourMapper->ScalarVisibilityOff();
	vtkSmartPointer<vtkActor> contourActor =
		vtkSmartPointer<vtkActor>::New();
	contourActor->SetMapper(contourMapper);
	contourActor->GetProperty()->SetOpacity(0.4);
	contourActor->GetProperty()->SetColor(1, 1, 1);
	contourActor->GetProperty()->SetRepresentationToSurface();

	// General
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	std::cout << "renderwindow: " << renderWindow << std::endl;
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderer->AddActor(trace1);
	renderer->AddActor(trace2);
	//renderer->AddActor(trace3);
	renderer->AddActor(contourActor);
	renderer->AddActor(outlineActor);

	renderer->AddActor(source3Actor);

	// Render and interact
	renderWindow->Render();

	// Initialize must be called prior to creating timer events.
	renderWindowInteractor->Initialize();
	/*
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> trackballMode = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	renderWindowInteractor->SetInteractorStyle(trackballMode);

	*/
	
	// Sign up to receive TimerEvent
	vtkSmartPointer<vtkCallBack> cb =
		vtkSmartPointer<vtkCallBack>::New();
	//cb->setup(source3, bounds[3], bounds[5]);
	//renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

	vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
	renderWindowInteractor->SetTimerEventId(vtkCommand::TimerEvent);
	int timerId = renderWindowInteractor->CreateRepeatingTimer(100);
	std::cout << "timerId: " << timerId << std::endl;

	// Start the interaction and timer
	renderWindowInteractor->Start();

	std::cout << "end of function" << std::endl;
}

int TestSebastienMartijn(int argc, char *argv[])
{
	std::string fileName;
	if (argc < 2)
	{
		std::cout << "Please specify the flow field file" << std::endl;
		return EXIT_FAILURE;
	}
	else
	{
		fileName = std::string("SMRX.vtk");
	}

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

	vtkSmartPointer<vtkPointSource> source1 = vtkSmartPointer<vtkPointSource>::New();
	vtkSmartPointer<vtkPointSource> source2 = vtkSmartPointer<vtkPointSource>::New();
	
	std::thread first (&ControllerProxy::run, ControllerProxy(source1, source2, renderWindow));
	FlowVisualiser visualiser(fileName, source1, source2, renderWindow);

	return EXIT_SUCCESS;
}
