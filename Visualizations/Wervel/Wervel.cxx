#include <vtkVersion.h>
#include <vtkPointSource.h>
#include <vtkKdTreePointLocator.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSliderWidget.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkWidgetEvent.h>
#include <vtkCallbackCommand.h>
#include <vtkWidgetEventTranslator.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSliderWidget.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkProperty.h>
#include <vtkMath.h>
#include <vtkConeSource.h>
#include <vtkImageReader.h>
#include <vtkBMPReader.h>
#include <vtkImageMapper.h>
#include <vtkInteractorStyleImage.h>
#include <vtkStructuredPointsReader.h>
#include <vtkImageDataGeometryFilter.h>

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

    vtkSmartPointer<vtkImageDataGeometryFilter> geometryFilter =
            vtkSmartPointer<vtkImageDataGeometryFilter>::New();
    geometryFilter->SetInputConnection(pointsReader->GetOutputPort());
    geometryFilter->Update();

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(geometryFilter->GetOutputPort());

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
    renderer->SetBackground(.3, .6, .3); // Background color green

    renderWindow->Render();
    renderWindowInteractor->Start();

    return EXIT_SUCCESS;
}