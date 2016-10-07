#include <vtkVersion.h>
#include <vtkStructuredPointsReader.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStreamTracer.h>
#include <vtkPointSource.h>
#include <vtkRungeKutta4.h>
#include <vtkTubeFilter.h>
#include <vtkStructuredPoints.h>
#include <vtkProperty.h>
#include <vtkOutlineFilter.h>
#include <vtkRungeKutta2.h>

std::string fileName;

int nPoints = 100;
int maxPropagation = 150;
double red[3] = {255, 0, 0};
double green[3] = {0, 255, 0};
double r = 15;

vtkSmartPointer<vtkActor> getStreamTraceActor(double x, double y, double z, double r, double color[], vtkAlgorithmOutput *flowfieldInput)
{
    // Generate random points
    vtkSmartPointer<vtkPointSource> pointSource = vtkSmartPointer<vtkPointSource>::New();
    pointSource->SetRadius(r);
    pointSource->SetNumberOfPoints(nPoints);
    pointSource->SetCenter(x, y, z);
    pointSource->Update();

    vtkSmartPointer<vtkRungeKutta2> integ = vtkSmartPointer<vtkRungeKutta2>::New();

    // Stream trace
    vtkSmartPointer<vtkStreamTracer> streamTracer = vtkSmartPointer<vtkStreamTracer>::New();
    streamTracer->SetInputConnection(flowfieldInput);
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
    tubeActor->GetProperty()->SetColor(color);
    return tubeActor;
}

int main (int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Please specify the wervel file" << std::endl;
        return EXIT_FAILURE;
    }
    else
    {
        fileName = std::string(argv[1]);
    }
    // Read flowfield
    vtkStructuredPointsReader *pointsReader = vtkStructuredPointsReader::New();
    pointsReader->SetFileName(fileName.c_str());
    pointsReader->Update();

    double bounds[6];
    pointsReader->GetOutput()->GetBounds(bounds);
    for (int i = 0; i < 6; i++)
    {
        std::cout << bounds[i] << std::endl;
    }


    // Create the outline
    vtkSmartPointer<vtkOutlineFilter> outline =
            vtkSmartPointer<vtkOutlineFilter>::New();
    outline->SetInputConnection(pointsReader->GetOutputPort());

    // Create traces
    vtkSmartPointer<vtkActor> trace1 = getStreamTraceActor(0, bounds[3]*0.25, bounds[5]/2, r, red, pointsReader->GetOutputPort());
    vtkSmartPointer<vtkActor> trace2 = getStreamTraceActor(0, bounds[3]*0.75, bounds[5]/2, r, green, pointsReader->GetOutputPort());

    //Map outline
    vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    outlineMapper->SetInputConnection(outline->GetOutputPort());
    vtkSmartPointer<vtkActor> outlineActor =
            vtkSmartPointer<vtkActor>::New();
    outlineActor->SetMapper(outlineMapper);
    outlineActor->GetProperty()->SetColor(1,1,1);

    // General
    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);


    renderer->AddActor(trace1);
    renderer->AddActor(trace2);
    renderer->AddActor(outlineActor);

    renderWindow->Render();
    renderWindowInteractor->Start();

    return EXIT_SUCCESS;
}

