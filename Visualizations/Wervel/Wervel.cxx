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


int main (int, char *[])
{
    vtkBMPReader *imageReader = vtkBMPReader::New();
    imageReader->SetFileName("../Flow data/wervel.vtk");
    vtkSmartPointer<vtkImageMapper> imageMapper = vtkSmartPointer<vtkImageMapper>::New();
    imageMapper->SetInputData(imageReader->GetOutput());

    imageMapper->SetColorWindow(255);
    imageMapper->SetColorLevel(127.5);

    vtkSmartPointer<vtkActor2D> imageActor = vtkSmartPointer<vtkActor2D>::New();
    imageActor->SetMapper(imageMapper);
    imageActor->SetPosition(20, 20);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkInteractorStyleImage> styleImage = vtkSmartPointer<vtkInteractorStyleImage>::New();

    interactor->SetInteractorStyle(styleImage);
    interactor->SetRenderWindow(renderWindow);
    renderer->AddActor2D(imageActor);
    renderWindow->Render();
    interactor->Start();



    return EXIT_SUCCESS;
}
