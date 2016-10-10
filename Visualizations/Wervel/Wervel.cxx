#include <vtkVersion.h>
#include <vtkStructuredPointsReader.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStreamTracer.h>
#include <vtkPointSource.h>
#include <vtkTubeFilter.h>
#include <vtkStructuredPoints.h>
#include <vtkProperty.h>
#include <vtkOutlineFilter.h>
#include <vtkRungeKutta2.h>
#include <vtkThreshold.h>
#include <vtkContourFilter.h>
#include <thread>
#include <vtkInteractorStyleTrackballCamera.h>

class vtkCallBack : public vtkCommand {
public:
    static vtkCallBack *New()
    {
        return new vtkCallBack;
    }

    virtual void Execute(vtkObject *caller, unsigned long eventId, void* vtkNotUsed(cellData))
    {
        if (eventId == vtkCommand::TimerEvent || eventId == vtkCommand::InteractionEvent)
        {
            std::cout << "animation step " << iteration << std::endl;
            double radius = 10;

            x = sin(2.0 * M_PI * iteration / 100)*radius + centerX;
            y = cos(2.0 * M_PI * iteration / 100)*radius + centerY;

            pointSource->SetCenter(0, x, y);

            iteration = (iteration+1) % 100;

            vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
            iren->GetRenderWindow()->Render();
        }
    }

    void setup(vtkSmartPointer<vtkPointSource> pointSource, double x, double y)
    {
        this->pointSource = pointSource;
        centerX = x*0.5;
        centerY = y*0.5;

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

    FlowVisualiser(std::string fileName);
    vtkSmartPointer<vtkActor> getStreamTraceActor(const double color[],
                                                  vtkAlgorithmOutput *flowFieldInput,
                                                  vtkSmartPointer<vtkPointSource> pointSource,
                                                  vtkSmartPointer<vtkStreamTracer> streamTracer);

    vtkSmartPointer<vtkPointSource> getPointSource(double x, double y, double z, double r, int nPoints);
};

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

FlowVisualiser::FlowVisualiser(std::string fileName) :
    red{255, 0, 0}, green{0, 255, 0}, blue{0, 0, 255}
{
    // Read flow field
    vtkStructuredPointsReader *pointsReader = vtkStructuredPointsReader::New();
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
    vtkSmartPointer<vtkPointSource> source1 = getPointSource(0, bounds[3]*0.25, bounds[5]/2, r, nPoints);
    vtkSmartPointer<vtkPointSource> source2 = getPointSource(0, bounds[3]*0.75, bounds[5]/2, r, nPoints);
    vtkSmartPointer<vtkPointSource> source3 = getPointSource(0, bounds[3]*0.5, bounds[5]/2, 0, 1);

    // Create traces
    vtkSmartPointer<vtkStreamTracer> streamTracer1;
    vtkSmartPointer<vtkStreamTracer> streamTracer2;
    vtkSmartPointer<vtkStreamTracer> streamTracer3;

    vtkSmartPointer<vtkActor> trace1 = getStreamTraceActor(red, pointsReader->GetOutputPort(), source1, streamTracer1);
    vtkSmartPointer<vtkActor> trace2 = getStreamTraceActor(green, pointsReader->GetOutputPort(),source2, streamTracer2);
    vtkSmartPointer<vtkActor> trace3 = getStreamTraceActor(blue, pointsReader->GetOutputPort(),source3, streamTracer3);

    //Map outline
    vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    outlineMapper->SetInputConnection(outline->GetOutputPort());
    vtkSmartPointer<vtkActor> outlineActor =
            vtkSmartPointer<vtkActor>::New();
    outlineActor->SetMapper(outlineMapper);
    outlineActor->GetProperty()->SetColor(1,1,1);

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
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderer->AddActor(trace1);
    renderer->AddActor(trace2);
    renderer->AddActor(trace3);
    renderer->AddActor(contourActor);
    renderer->AddActor(outlineActor);

    // Render and interact
    renderWindow->Render();

    // Initialize must be called prior to creating timer events.
    renderWindowInteractor->Initialize();
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> trackballMode = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    renderWindowInteractor->SetInteractorStyle(trackballMode);

    // Sign up to receive TimerEvent
    vtkSmartPointer<vtkCallBack> cb =
            vtkSmartPointer<vtkCallBack>::New();
    cb->setup(source3, bounds[3], bounds[5]);
    renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

    int timerId = renderWindowInteractor->CreateRepeatingTimer(1);
    std::cout << "timerId: " << timerId << std::endl;

    // Start the interaction and timer
    renderWindowInteractor->Start();

}

int main (int argc, char *argv[])
{
    std::string fileName;
    if (argc < 2)
    {
        std::cout << "Please specify the flow field file" << std::endl;
        return EXIT_FAILURE;
    }
    else
    {
        fileName = std::string(argv[1]);
    }

    FlowVisualiser visualiser(fileName);

    return EXIT_SUCCESS;
}