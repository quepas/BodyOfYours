#include "MainWindow.h"

#include <QtWidgets/QLabel>
#include <QtWidgets/QAction>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QToolBar>
#include <QtCore/QTimer>

#include <RecFusion.h>

#include <iostream>
#include <fstream>

using namespace RecFusion;


MainWindow::MainWindow() :
	m_imgLabel(0),
	m_recLabel(0),
	m_sensor(0),
	m_rec(0),
	m_imgColor(0),
	m_imgScene(0),
	m_imgDepth(0),
	m_reconstruct(false)
{
	m_imgLabel = new QLabel;
	m_recLabel = new QLabel;
	QHBoxLayout* l = new QHBoxLayout;
	l->addWidget(m_imgLabel);
	l->addWidget(m_recLabel);

	QWidget* wt = new QWidget;
	wt->setLayout(l);
	setCentralWidget(wt);

	// Output RecFusion SDK version
	std::cout << "Using RecFusionSDK " << RecFusionSDK::majorVersion() << "." << RecFusionSDK::minorVersion() << std::endl;

	// Load license file
	bool ok = RecFusionSDK::setLicenseFile("License.dat");
	if (!ok)
		std::cout << "Invalid RecFusion license. Export will be disabled." << std::endl;

	// Open sensor
	m_sensor = new Sensor;

	// Get number of sensors
	int numSensors = m_sensor->deviceCount();
	// Open first sensor if available
	if (numSensors == 0)
		ok = false;
	else
	{
		char deviceName[50];
		int ret = m_sensor->deviceName(0, deviceName, sizeof(deviceName));
		if (ret >= 0)
			std::cout << "Using sensor: " << deviceName << std::endl;
		ok = m_sensor->open(0);
	}
	if (!ok)
	{
		delete m_sensor;
		m_sensor = 0;
		std::cout << "Couldn't open sensor" << std::endl;
		QTimer::singleShot(0,this,SLOT(close()));
		return;
	}

	int w = m_sensor->width();
	int h = m_sensor->height();
	
	// Create buffers for color and depth images
	m_imgColor = new ColorImage(w, h);
	m_imgDepth = new DepthImage(w, h);
	m_imgScene = new ColorImage(w, h);

	QToolBar* toolbar = new QToolBar(this);
	addToolBar(toolbar);

	QAction* a;

	a = new QAction("Start reconstruction",this);
	a->setShortcut(QKeySequence("F5"));
	connect(a,SIGNAL(triggered()),this,SLOT(onStartReconstruction()));
	addAction(a);
	toolbar->addAction(a);

	a = new QAction("Stop reconstruction", this);
	a->setShortcut(QKeySequence("F6"));
	connect(a,SIGNAL(triggered()),this,SLOT(onStopReconstruction()));
	addAction(a);
	toolbar->addAction(a);

	QTimer* timer = new QTimer(this);
	connect(timer,SIGNAL(timeout()),this,SLOT(processFrame()));
	timer->start(50);

	std::cout << std::endl << "Press F5 to start reconstruction and F6 to stop reconstruction." << std::endl << std::endl;
}


MainWindow::~MainWindow()
{
	if (m_sensor)
		m_sensor->close();

	delete m_imgColor;
	delete m_imgDepth;
	delete m_imgScene;
	delete m_sensor;
	delete m_rec;
}


void MainWindow::onStartReconstruction()
{
	m_reconstruct = false;

	delete m_rec;
	m_rec = 0;

	// Set reconstruction parameters
	ReconstructionParams params;
	params.setImageSize(m_sensor->width(), m_sensor->height());
	params.setIntrinsics(m_sensor->depthIntrinsics());
	params.setVolumePosition(Vec3(0, 0, 1000));
	params.setVolumeResolution(Vec3i(256, 256, 256));
	params.setVolumeSize(Vec3(1000,1000,1000));
	
	// Create reconstruction object
	m_rec = new Reconstruction(params);

	m_reconstruct = true;
}


void MainWindow::onStopReconstruction()
{
	m_reconstruct = false;
	if (!m_rec)
		return;

	Mesh mesh;
	bool ok = m_rec->getMesh(&mesh);
	delete m_rec;
	m_rec = 0;
	if (!ok)
	{
		std::cout << "Couldn't retrieve mesh" << std::endl;
		return;
	}

	std::cout << "Reconstructed mesh (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
  MeshViewer* viewer = new MeshViewer;
  //viewer.showMesh(&mesh);
	ok = mesh.save("mesh.ply",Mesh::PLY);
	if (ok)
	std::cout << "Saved mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;

#ifndef _DEBUG
	// Show mesh in viewer
	MeshViewer viewer;
	viewer.showMesh(&mesh);
#endif

	ok = mesh.decimate(0.1,1.5,true);
	if (ok)
	{
		ok = mesh.save("mesh_decimated.ply",Mesh::PLY);
		if (ok)
			std::cout << "Saved decimated mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
		ok = mesh.save("mesh_decimated.obj",Mesh::OBJ);
		if (ok)
			std::cout << "Saved decimated mesh as OBJ (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
		ok = mesh.save("mesh_decimated.stl",Mesh::STL);
		if (ok)
			std::cout << "Saved decimated mesh as STL (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
	}

	// Save as Wavefront OBJ by accessing mesh data directly
	std::ofstream out("mesh.obj");
	for (int i = 0; i < (int)mesh.vertexCount(); ++i)
	{
		Mesh::Coordinate v, n;
		Mesh::Color c;
		v = mesh.vertex(i);
		n = mesh.normal(i);
		c = mesh.color(i);
		out << "vn " << n.x << " " << -n.y << " " << -n.z << std::endl;
		out << "v " << v.x << " " << -v.y << " " << -v.z << " " << c.r << " " << c.g << " " << c.b << std::endl;
	}

	for (int i = 0; i < (int)mesh.triangleCount(); ++i)
	{
		Mesh::Triangle t;
		t = mesh.triangle(i);
		out << "f " << t.i1+1 << "//" << t.i1+1 << " "
			<< t.i2+1 << "//" << t.i2+1 << " "
			<< t.i3+1 << "//" << t.i3+1 << std::endl;
	}
	out.close();

	Vec3 center = mesh.center();
	Vec3 extent = mesh.extent();
	extent[0] /= 2.0;
	extent[1] /= 2.0;
	extent[2] /= 2.0;
	ok = mesh.crop(center, extent);
	if (ok)
	{
		ok = mesh.save("mesh_cropped.ply", Mesh::PLY);
		if (ok)
			std::cout << "Saved cropped mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
	}

	ok = mesh.isManifold();
	if (ok)
	{
		std::cout << "Mesh is manifold" << std::endl;
	}
	else
	{
		std::cout << "Mesh is not manifold. Closing holes ..." << std::endl;
		ok = mesh.fillHoles();
		if (ok)
		{
			std::cout << "Closed holes." << std::endl;
			ok = mesh.save("mesh_closed.ply",Mesh::PLY);
			if (ok)
				std::cout << "Saved closed mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
		}
	}
}


void MainWindow::processFrame()
{
	bool ok = m_sensor->readImage(*m_imgDepth,*m_imgColor);
	if (ok)
	{
		if (m_reconstruct)
		{
			bool status;
			// Add frame to reconstruction
			m_rec->addFrame(0,*m_imgDepth,*m_imgColor,&status,m_imgScene);

			// Display rendering of current reconstruction
			QImage image(m_imgScene->data(), m_imgScene->width(), m_imgScene->height(), QImage::Format_RGB888);
			m_recLabel->setPixmap(QPixmap::fromImage(image).scaled(m_imgScene->width(), m_imgScene->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
			if (!status)
				std::cout << "Lost track" << std::endl;
		}

		// Display color image
		QImage image(m_imgColor->data(), m_imgColor->width(), m_imgColor->height(), QImage::Format_RGB888);
		m_imgLabel->setPixmap(QPixmap::fromImage(image).scaled(m_imgColor->width(), m_imgColor->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
		update();
	}
}
