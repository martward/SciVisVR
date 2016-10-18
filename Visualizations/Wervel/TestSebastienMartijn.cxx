/*=========================================================================

  Program:   Visualization Toolkit

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include <Windows.h>
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
#include "vtkOpenVRRenderWindowInteractor.h"

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
#include <vtkSphereSource.h>

//#include "Animation.h"

#include <thread>         // std::thread
#include <chrono>

std::string fileName;



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
	//FlowVisualiser(std::string fileName, vtkSmartPointer<vtkPointSource> source3, vtkSmartPointer<vtkPointSource> source4, AnimationTicker &animation);
	FlowVisualiser(std::string fileName, vtkSmartPointer<vtkPointSource> source3, vtkSmartPointer<vtkPointSource> source4);
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
	streamTracer->SetInputConnection(flowFieldInput);
	streamTracer->SetSourceConnection(pointSource->GetOutputPort());
	streamTracer->SetMaximumPropagation(maxPropagation);
	streamTracer->SetInitialIntegrationStep(0.1);
	streamTracer->SetIntegrationDirectionToForward();
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
//FlowVisualiser::FlowVisualiser(std::string fileName, vtkSmartPointer<vtkPointSource> source3, vtkSmartPointer<vtkPointSource> source4, AnimationTicker &animation) :
FlowVisualiser::FlowVisualiser(std::string fileName, vtkSmartPointer<vtkPointSource> source3, vtkSmartPointer<vtkPointSource> source4):
	red{ 255, 0, 0 }, green{ 0, 255, 0 }, blue{ 0, 0, 255 }
{

	// General
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	std::cout << "renderwindow: " << renderWindow << std::endl;
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkOpenVRRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkOpenVRRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

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
	vtkSmartPointer<vtkStreamTracer> streamTracer1 = vtkSmartPointer<vtkStreamTracer>::New();
	vtkSmartPointer<vtkStreamTracer> streamTracer2 = vtkSmartPointer<vtkStreamTracer>::New();
	vtkSmartPointer<vtkStreamTracer> streamTracer3 = vtkSmartPointer<vtkStreamTracer>::New();

	std::vector<vtkSmartPointer<vtkStreamTracer>> tracers;
	tracers.insert(tracers.end(), streamTracer1);
	tracers.insert(tracers.end(), streamTracer2);
	tracers.insert(tracers.end(), streamTracer3);

	vtkSmartPointer<vtkActor> trace1 = getStreamTraceActor(red, pointsReader->GetOutputPort(), source1, streamTracer1);
	vtkSmartPointer<vtkActor> trace2 = getStreamTraceActor(green, pointsReader->GetOutputPort(), source2, streamTracer2);
	vtkSmartPointer<vtkActor> trace3 = getStreamTraceActor(blue, pointsReader->GetOutputPort(), source3, streamTracer3);

	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetRadius(1);
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
	sphereMapper->ScalarVisibilityOff();

	std::vector<vtkSmartPointer<vtkActor>> tracePointActors;
	int nActors = (int)streamTracer1->GetOutput()->GetNumberOfLines() +
		(int)streamTracer2->GetOutput()->GetNumberOfLines() +
		(int)streamTracer3->GetOutput()->GetNumberOfLines();

	double * color = (double*)red;
	for (int i = 0; i < nActors; i++)
	{
		vtkSmartPointer<vtkActor> pointActor = vtkSmartPointer<vtkActor>::New();
		pointActor->SetMapper(sphereMapper);
		pointActor->GetProperty()->SetColor(color);
		renderer->AddActor(pointActor);
		tracePointActors.insert(tracePointActors.end(), pointActor);

		if (i == streamTracer1->GetOutput()->GetNumberOfLines() - 1)
		{
			color = (double *)green;
		}
		else if (i == streamTracer1->GetOutput()->GetNumberOfLines() + streamTracer2->GetOutput()->GetNumberOfLines() - 1)
		{
			std::cout << "create blue actor" << std::endl;
			color = (double *)blue;
		}
	}
	std::cout << tracePointActors.size() << " in actor vector" << std::endl;

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

	//renderer->AddActor(trace1);
	//renderer->AddActor(trace2);
	renderer->AddActor(trace3);
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
	cb->setup(source3, bounds[3], bounds[5]);
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

	vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
	//renderWindowInteractor->SetTimerEventId(vtkCommand::TimerEvent);
	int timerId = renderWindowInteractor->CreateRepeatingTimer(100);
	std::cout << "timerId: " << timerId << std::endl;

	/*
	vtkOpenVRRenderWindowInteractor *inter = vtkOpenVRRenderWindowInteractor::New();
	inter->CreateRepeatingTimer(10);
	*/
	
	AnimationTicker animation = AnimationTicker();
	renderWindowInteractor->SetAnimation(animation);
	std::cout << "animation1: " << &animation << std::endl;
	animation.source1 = source3;
	animation.centerX = bounds[3] / 2;
	animation.centerY = bounds[5] / 2;
	animation.tracers = tracers;
	animation.actors = tracePointActors;

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

	

	vtkSmartPointer<vtkPointSource> source1 = vtkSmartPointer<vtkPointSource>::New();
	vtkSmartPointer<vtkPointSource> source2 = vtkSmartPointer<vtkPointSource>::New();
	
	//AnimationTicker animation = AnimationTicker();
	//FlowVisualiser visualiser(fileName, source1, source2, animation);
	FlowVisualiser visualiser(fileName, source1, source2);

	return EXIT_SUCCESS;
}
