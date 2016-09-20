#include <vtkVersion.h>
#include <vtkStructuredPointsReader.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStreamTracer.h>
#include <vtkStructuredGridReader.h>
#include <vtkPointSource.h>
#include <vtkRungeKutta4.h>
#include <vtkTubeFilter.h>


std::string fileName;

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
    vtkStructuredPointsReader *pointsReader = vtkStructuredPointsReader::New();
    pointsReader->SetFileName(fileName.c_str());
    pointsReader->Update();

    vtkSmartPointer<vtkPointSource> pointSource = vtkSmartPointer<vtkPointSource>::New();
    pointSource->SetRadius(10.0);
    pointSource->SetNumberOfPoints(1000);

    vtkSmartPointer<vtkRungeKutta4> integ = vtkSmartPointer<vtkRungeKutta4>::New();


    vtkSmartPointer<vtkStreamTracer> streamTracer = vtkSmartPointer<vtkStreamTracer>::New();
    streamTracer->SetInputConnection(pointsReader->GetOutputPort());
    streamTracer->SetSourceConnection(pointSource->GetOutputPort());
    streamTracer->SetMaximumPropagation(100);
    streamTracer->SetInitialIntegrationStep(0.1);
    streamTracer->SetIntegrationDirectionToBoth();
    streamTracer->SetIntegrator(integ);

    vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter->SetInputConnection(streamTracer->GetOutputPort());
    tubeFilter->SetRadius(0.1);
    tubeFilter->SetNumberOfSides(50);
    tubeFilter->Update();

    /*
    vtkSmartPointer<vtkImageDataGeometryFilter> geometryFilter =
            vtkSmartPointer<vtkImageDataGeometryFilter>::New();
    geometryFilter->SetInputConnection(pointsReader->GetOutputPort());
    geometryFilter->Update();
     */

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tubeFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderer->AddActor(actor);

    renderWindow->Render();
    renderWindowInteractor->Start();

    return EXIT_SUCCESS;
}