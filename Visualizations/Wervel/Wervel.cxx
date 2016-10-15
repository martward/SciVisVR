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
#include <vtkSphereSource.h>

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
            int actorIndex = 0;
            for (int j = 0; j < tracers.size(); j++)
            {
                vtkSmartPointer<vtkStreamTracer> tracer = tracers.at(j);
                tracer->GetOutput()->GetLines()->InitTraversal();
                //std::cout << "tracer: " << j << std::endl;
                for (int i = 0; i < tracer->GetOutput()->GetNumberOfLines(); i++)
                {
                    vtkSmartPointer<vtkActor> actor = actors.at(actorIndex);
                    vtkIdType *pts;
                    vtkIdType npts;
                    tracer->GetOutput()->GetLines()->GetNextCell(npts, pts);
                    //std::cout << *pts << std::endl;

                    actor->SetPosition(tracer->GetOutput()->GetPoint((int)*pts + iterators[actorIndex] % (int)npts));

                    iterators[actorIndex] = (iterators[actorIndex] + 3) % (int)npts;
                    actorIndex++;
                }

            }


            vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
            iren->GetRenderWindow()->Render();
            //std::cout << "Lines: " << tracer->GetOutput()->GetNumberOfLines() << std::endl;

            /*
            vtkSmartPointer<vtkActor> actor = actors.at(iteration % actors.size());
            actor->SetPosition(tracer->GetOutput()->GetPoint(iteration));

            iteration = (iteration + 5) % (int)tracer->GetOutput()->GetNumberOfPoints();
            vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
            iren->GetRenderWindow()->Render();
             */

        }
    }

    void setup(std::vector<vtkSmartPointer<vtkActor>> actors, std::vector<vtkSmartPointer<vtkStreamTracer>> tracers)
    {
        this->tracers = tracers;
        this->actors = actors;

        int iterators[actors.size()];

        /*
        std::cout << "nlines: " << tracer->GetOutput()->GetNumberOfLines() << std::endl;
        tracer->GetOutput()->GetLines()->InitTraversal();
        for (int i = 0; i < tracer->GetOutput()->GetNumberOfLines(); i++)
        {
            vtkIdType *pts;
            vtkIdType npts;
            int s = tracer->GetOutput()->GetLines()->GetNextCell(npts, pts);
            std::cout << npts << std::endl;
        }
        */
    }

private:
    std::vector<vtkSmartPointer<vtkStreamTracer>> tracers;
    std::vector<vtkSmartPointer<vtkActor>> actors;
    int iterators[200];
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

    // Stream tracer
    streamTracer->SetInputConnection(flowFieldInput);
    streamTracer->SetSourceConnection(pointSource->GetOutputPort());
    streamTracer->SetMaximumPropagation(maxPropagation);
    streamTracer->SetInitialIntegrationStep(0.1);
    streamTracer->SetIntegrationDirectionToForward();
    streamTracer->SetIntegrator(integ);
    streamTracer->SetComputeVorticity(false);
    streamTracer->SetSurfaceStreamlines(false);
    streamTracer->Update();

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

    // General
    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

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
    vtkSmartPointer<vtkStreamTracer> streamTracer1 = vtkSmartPointer<vtkStreamTracer>::New();
    vtkSmartPointer<vtkStreamTracer> streamTracer2 = vtkSmartPointer<vtkStreamTracer>::New();
    vtkSmartPointer<vtkStreamTracer> streamTracer3 = vtkSmartPointer<vtkStreamTracer>::New();

    std::vector<vtkSmartPointer<vtkStreamTracer>> tracers;
    tracers.insert(tracers.end(), streamTracer1);
    tracers.insert(tracers.end(), streamTracer2);
    tracers.insert(tracers.end(), streamTracer3);

    vtkSmartPointer<vtkActor> trace1 = getStreamTraceActor(red, pointsReader->GetOutputPort(), source1, streamTracer1);
    vtkSmartPointer<vtkActor> trace2 = getStreamTraceActor(green, pointsReader->GetOutputPort(),source2, streamTracer2);
    vtkSmartPointer<vtkActor> trace3 = getStreamTraceActor(blue, pointsReader->GetOutputPort(),source3, streamTracer3);

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
        tracePointActors.insert (tracePointActors.end(), pointActor);
        std::cout << "create actor" << std::endl;

        if (i == streamTracer1->GetOutput()->GetNumberOfLines()-1)
        {
            color = (double *)green;
        }
        else if (i == streamTracer1->GetOutput()->GetNumberOfLines() + streamTracer2->GetOutput()->GetNumberOfLines()-1)
        {
            std::cout << "create blue actor" << std::endl;
            color = (double *)blue;
        }
    }
    std::cout << tracePointActors.size() << " in actor vector" << std::endl;


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

    //renderer->AddActor(trace1);
    //renderer->AddActor(trace2);
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
    cb->setup(tracePointActors, tracers);
    renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

    int timerId = renderWindowInteractor->CreateRepeatingTimer(1000/90);
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