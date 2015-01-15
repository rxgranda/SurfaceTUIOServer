//============================================================================================-----
//== NaturalPoint Tracking Tools API Sample: Accessing Camera, Marker, and Trackable Information
//==
//== This command-line application loads a Tracking Tools Project, lists cameras, and 3d marker
//== count.
//============================================================================================-----

#include <windows.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stddef.h>
#include "Pluma.h"

// For OpenSceneGraph
////////////////////////////////////////////
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile> 
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/ShapeDrawable>
#include <iostream>
#include <osgText/Text>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
//////////////////////////////////////////// 

////////////////////////////////////////////for algebra
#include <gmtl/gmtl.h>
#include <gmtl/Matrix.h>

////////////////////////////////////////////
#include "NPTrackingTools.h"
/////////////////////////////////////// for TUIO
#include "TuioServer.h"
#include "TuioCursor.h"
#include <SDL.h>
#include <SDL_thread.h>

#define MAX_PEN_COUNT 15;
using namespace TUIO;


/////////////////////////////////

using namespace cv;
using namespace cti;
////////////////////////////////////////////
//== Callback prototype ==--

float positionX=10.0f,positionY=4.0f,positionZ=-10.0f;
float positionX_final=0.0f,positionY_final=0.0f,positionZ_final=0.0f;
float positionX_esferaEstatica=0.0f,positionY_esferaEstatica=0.0f,positionZ_esferaEstatica=0.0f;
float positionCaidaX=0.0f,positionCaidaY=0.0f,positionCaidaZ=0.0f;
bool choque=false,toca=false;
float dirx=0.0,diry=0.0,dirz=0.0,r=0.0;
/////////////////////////////////////////////

///////////////// CALIBRACION//////////////////////
int numMuestra,numMuestra2=0;
gmtl::Vec3f promedio, muestra1,muestra2,muestra3;
gmtl::Vec3f promedio2, muestra21,muestra22,muestra23;
bool  mCalibracion = false;
int indexEsquina,indexFlag;
struct posicion {
	float x; float y; float z;
};
posicion esquina1, esquina2, esquina3, esquina4;
gmtl::Vec3f esquinaA, esquinaB, esquinaC, esquinaD;
gmtl::Vec3f xVecCor, yVecCor, zVecCor;
//Cambio
//gmtl::Matrix33f C_inv,C,C_alt;
//float D;
//cv::Mat lambda= cv::Mat::zeros(3, 3, CV_32FC1);
//cambio
//cv::Mat * lambda = new cv::Mat( cv::Mat::zeros(3, 3, CV_32F) );
Point2f inputQuad[4]; 
// Output Quadilateral or World plane coordinates
Point2f outputQuad[4];

//float Pluma::WIDTH=7680;float Pluma::HEIGHT=4320;

gmtl::Vec3f coordenadasLocalesA;
gmtl::Vec3f coordenadasLocalesB;
gmtl::Vec3f coordenadasLocalesC;
gmtl::Vec3f coordenadasLocalesD;
gmtl::Vec3f coordenadasLocales;
gmtl::Vec3f coordenadasLocales2;
//gmtl::Vec3f  xlocal,zlocal,ylocal;
//float divisor_;
///////////////// CALIBRACION//////////////////////



///////////////TUIO//////////////
TuioServer *tuioServer;
TuioCursor *cursor,*cursor2;
long idCursor1=-1000, idCursor2=-1000;
TuioTime currentTime;

///////////////////////////////

// Local function prototypes
void CheckResult( NPRESULT result );

// Local constants
const float kRadToDeg = 0.0174532925f;

// Local class definitions
class Point4
{
public:
	Point4( float x, float y, float z, float w );

	float           operator[]( int idx ) const { return mData[idx]; }
	const float*    Data() const { return mData; }

private:
	float           mData[4];
};

class TransformMatrix
{
public:
	TransformMatrix();

	TransformMatrix( float m11, float m12, float m13, float m14,
		float m21, float m22, float m23, float m24,
		float m31, float m32, float m33, float m34,
		float m41, float m42, float m43, float m44 );

	void            SetTranslation( float x, float y, float z );
	void            Invert();

	TransformMatrix operator*( const TransformMatrix &rhs );
	Point4          operator*( const Point4 &v );

	static TransformMatrix RotateX( float rads );
	static TransformMatrix RotateY( float rads );
	static TransformMatrix RotateZ( float rads );

private:
	float           mData[4][4];
};

// Clase para eventos del teclado 

class myKeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
	virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
	virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };
};

bool myKeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
	switch(ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			switch(ea.getKey())
			{
			case 'r':
				choque=false;
				toca=false;
				return false;
				break;
			default:
				return false;
			} 
		}
	default:
		return false;
	}
}


DWORD WINAPI iniciarOSG(LPVOID lpParam){
	osgViewer::Viewer viewer;
	viewer.setUpViewInWindow(400, 400, 640, 480); // Set del Size de la Ventana de la Escena.


	// Declaro a un Grupo como la Raiz de la Escena.
	osg::Group* root = new osg::Group();

	// Creo la ESFERA ESTATICA 
	osg::Sphere* esferaEstatica = new osg::Sphere( osg::Vec3(0,0,0), 2.0);
	osg::ShapeDrawable* esferaEstaticaDrawable = new osg::ShapeDrawable(esferaEstatica);
	esferaEstaticaDrawable->setColor( osg::Vec4(0.1, 255.0, 0.1, 0.1) );

	osg::PositionAttitudeTransform* esferaEstaticaXForm = new osg::PositionAttitudeTransform();
	esferaEstaticaXForm->setPosition(osg::Vec3(positionX_esferaEstatica,positionY_esferaEstatica,positionZ_esferaEstatica));

	osg::Geode* esferaEstaticaGeode = new osg::Geode();
	//root->addChild(esferaEstaticaXForm);
	esferaEstaticaXForm->addChild(esferaEstaticaGeode);
	esferaEstaticaGeode->addDrawable(esferaEstaticaDrawable);


	/*  osg::ref_ptr<osg::Node> puntoRef = osgDB::readNodeFile( "axes.osgt" );
	root->addChild(puntoRef);*/

	// Dibujo ESFERA en Movimiento
	osg::Sphere* unitSphere = new osg::Sphere( osg::Vec3(0,0,0), 0.3);
	osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
	unitSphereDrawable->setColor( osg::Vec4(0.0, 0.0, 0.0, 1.0) );

	osg::PositionAttitudeTransform* unitSphereXForm = 
		new osg::PositionAttitudeTransform();
	unitSphereXForm->setPosition(osg::Vec3(positionX,positionY,positionZ));
	osg::Geode* unitSphereGeode = new osg::Geode();
	unitSphereGeode->addDrawable(unitSphereDrawable);
	unitSphereXForm->addChild(unitSphereGeode);
	root->addChild(unitSphereXForm);
	//////////////////////////////////////////////////////////////
	// Esfera Punta A
	osg::Geode* esquinaAGeode = new osg::Geode();
	osg::Sphere* esquinaASphere= new osg::Sphere( osg::Vec3(0,0,0), 0.1);
	osg::ShapeDrawable* esquinaADrawable = new osg::ShapeDrawable(esquinaASphere);
	esquinaADrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 1.0) );
	esquinaAGeode->addDrawable(esquinaADrawable);
	osg::PositionAttitudeTransform* esquinaAXForm = new osg::PositionAttitudeTransform();	
	gmtl::Vec3f coor=Pluma::C*coordenadasLocalesA+esquinaA;
	esquinaAXForm->setPosition(osg::Vec3(coor[0],coor[1],coor[2])); 
	esquinaAXForm->addChild(esquinaAGeode);
	root->addChild(esquinaAXForm);

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	// Esfera Punta B
	osg::Geode* esquinaBGeode = new osg::Geode();
	osg::Sphere* esquinaBSphere= new osg::Sphere( osg::Vec3(0,0,0), 0.1);
	osg::ShapeDrawable* esquinaBDrawable = new osg::ShapeDrawable(esquinaBSphere);
	esquinaBDrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 1.0) );
	esquinaBGeode->addDrawable(esquinaBDrawable);
	osg::PositionAttitudeTransform* esquinaBXForm = new osg::PositionAttitudeTransform();
	
	gmtl::Vec3f coorB=esquinaA+Pluma::C*coordenadasLocalesB;
	esquinaBXForm->setPosition(osg::Vec3(coorB[0],coorB[1],coorB[2])); 
	esquinaBXForm->addChild(esquinaBGeode);
	root->addChild(esquinaBXForm);

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	// Esfera Punta C
	osg::Geode* esquinaCGeode = new osg::Geode();
	osg::Sphere* esquinaCSphere= new osg::Sphere( osg::Vec3(0,0,0), 0.1);
	osg::ShapeDrawable* esquinaCDrawable = new osg::ShapeDrawable(esquinaCSphere);
	esquinaCDrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 1.0) );
	esquinaCGeode->addDrawable(esquinaCDrawable);
	osg::PositionAttitudeTransform* esquinaCXForm = new osg::PositionAttitudeTransform();
	
	gmtl::Vec3f coorC=esquinaA+Pluma::C*coordenadasLocalesC;
	esquinaCXForm->setPosition(osg::Vec3(coorC[0],coorC[1],coorC[2])); 
	esquinaCXForm->addChild(esquinaCGeode);
	root->addChild(esquinaCXForm);

	//////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	// Esfera Punta D
	osg::Geode* esquinaDGeode = new osg::Geode();
	osg::Sphere* esquinaDSphere= new osg::Sphere( osg::Vec3(0,0,0), 0.1);
	osg::ShapeDrawable* esquinaDDrawable = new osg::ShapeDrawable(esquinaDSphere);
	esquinaDDrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 1.0) );
	esquinaDGeode->addDrawable(esquinaDDrawable);
	osg::PositionAttitudeTransform* esquinaDXForm = new osg::PositionAttitudeTransform();
	
	gmtl::Vec3f coorD=esquinaA+Pluma::C*coordenadasLocalesD;
	esquinaDXForm->setPosition(osg::Vec3(coorD[0],coorD[1],coorD[2])); 
	esquinaDXForm->addChild(esquinaDGeode);
	root->addChild(esquinaDXForm);

	//////////////////////////////////////////////////////////////

	// Esfera direccion
	osg::Geode* puntoDireccionGeode = new osg::Geode();
	osg::Sphere* puntoDireccion = new osg::Sphere( osg::Vec3(0,0,0), 0.1);
	osg::ShapeDrawable* puntoDireccionDrawable = new osg::ShapeDrawable(puntoDireccion);
	puntoDireccionDrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 1.0) );
	puntoDireccionGeode->addDrawable(puntoDireccionDrawable);

	osg::PositionAttitudeTransform* puntoDireccionXForm = new osg::PositionAttitudeTransform();
//
	gmtl::Vec3f coorL=esquinaA+Pluma::C*coordenadasLocales;
	//
	puntoDireccionXForm->setPosition(osg::Vec3(coorL[0],coorL[1],coorL[2])); 


	puntoDireccionXForm->addChild(puntoDireccionGeode);


	root->addChild(puntoDireccionXForm);



	// Dibujo Linea
	osg::Vec3 sp(0.0,0.0,0.0); 
	osg::Vec3 pX(3.0,0.0,0.0);
	osg::Vec3 pY(0.0,3.0,0.0);
	osg::Vec3 pZ(0.0,0.0,3.0);

	osg::ref_ptr<osg::Geometry> ejeX( new osg::Geometry); 
	osg::ref_ptr<osg::Geometry> ejeY( new osg::Geometry);
	osg::ref_ptr<osg::Geometry> ejeZ( new osg::Geometry); 

	osg::ref_ptr<osg::Vec3Array> pointsX = new osg::Vec3Array; 
	pointsX->push_back(sp); 
	pointsX->push_back(pX);
	osg::ref_ptr<osg::Vec3Array> pointsY = new osg::Vec3Array; 
	pointsY->push_back(sp); 
	pointsY->push_back(pY);
	osg::ref_ptr<osg::Vec3Array> pointsZ = new osg::Vec3Array; 
	pointsZ->push_back(sp); 
	pointsZ->push_back(pZ);

	osg::ref_ptr<osg::Vec4Array> colorX = new osg::Vec4Array; 
	colorX->push_back(osg::Vec4(1.0,0.0,0.0,1.0)); 
	osg::ref_ptr<osg::Vec4Array> colorY = new osg::Vec4Array; 
	colorY->push_back(osg::Vec4(0.0,1.0,0.0,1.0));
	osg::ref_ptr<osg::Vec4Array> colorZ = new osg::Vec4Array; 
	colorZ->push_back(osg::Vec4(0.0,0.0,1.0,1.0)); 

	ejeX->setVertexArray(pointsX.get()); 
	ejeX->setColorArray(colorX.get()); 
	ejeY->setVertexArray(pointsY.get()); 
	ejeY->setColorArray(colorY.get()); 
	ejeZ->setVertexArray(pointsZ.get()); 
	ejeZ->setColorArray(colorZ.get()); 

	ejeX->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE); 
	ejeX->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
	ejeY->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE); 
	ejeY->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
	ejeZ->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE); 
	ejeZ->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));



	osg::ref_ptr<osg::Geode> geodeAxe (new osg::Geode);

	geodeAxe->addDrawable(ejeX);
	geodeAxe->addDrawable(ejeY);
	geodeAxe->addDrawable(ejeZ);

	osg::ref_ptr<osgText::Text> labX (new osgText::Text()) ;
	osg::ref_ptr<osgText::Text> labY (new osgText::Text()) ;
	osg::ref_ptr<osgText::Text> labZ (new osgText::Text()) ;

	labX->setText("+X");
	labY->setText("+Y");
	labZ->setText("+Z");

	labX->setPosition(pX);
	labY->setPosition(pY);
	labZ->setPosition(pZ);
	labX->setCharacterSize(0.5);
	labY->setCharacterSize(0.5);
	labZ->setCharacterSize(0.5);

	geodeAxe->addDrawable(labX);
	geodeAxe->addDrawable(labY);
	geodeAxe->addDrawable(labZ);

	root->addChild(geodeAxe);
	// Dibujo PLANO PISO
	// Declaro los vertices del Piso
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	//vertices->push_back( osg::Vec3(-20.0f, -20.0f,0.0f ) );
	//vertices->push_back( osg::Vec3(20.0f, -20.0f, 0.0f) );
	//vertices->push_back( osg::Vec3(20.0f,20.0f, 0.0f) );
	//vertices->push_back( osg::Vec3(-20.0f, 20.0f, 0.0f) );
	vertices->push_back( osg::Vec3(0.0f, 0.0f,0.0f ) );
	vertices->push_back( osg::Vec3(6.44f, 0.0f, 0.0f) );
	vertices->push_back( osg::Vec3(6.44f,0.0f, -4.08f) );
	vertices->push_back( osg::Vec3(0.0f, 0.0f, -4.08f) );

	// Declaro la normal
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back( osg::Vec3(0.0f,1.0f, 0.0f) );

	// Colores en los vertices
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	//colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
	//colors->push_back( osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );
	//colors->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
	quad->setVertexArray( vertices.get() );
	quad->setNormalArray( normals.get() );
	quad->setNormalBinding( osg::Geometry::BIND_OVERALL );
	quad->setColorArray( colors.get() );
	quad->setColorBinding( osg::Geometry::BIND_OVERALL );

	quad->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );
	osg::ref_ptr<osg::Geode> rootPlane = new osg::Geode;
	rootPlane->addDrawable( quad.get() );



	///////////////////////////////////////////////////// Dibujar plano proy

	// Dibujo PLANO PISO
	// Declaro los vertices del Piso
	osg::ref_ptr<osg::Vec3Array> vertices2 = new osg::Vec3Array;
	//vertices->push_back( osg::Vec3(-20.0f, -20.0f,0.0f ) );
	//vertices->push_back( osg::Vec3(20.0f, -20.0f, 0.0f) );
	//vertices->push_back( osg::Vec3(20.0f,20.0f, 0.0f) );
	//vertices->push_back( osg::Vec3(-20.0f, 20.0f, 0.0f) );
	vertices2->push_back( osg::Vec3(esquinaA[0],esquinaA[1],esquinaA[2]) );
	vertices2->push_back( osg::Vec3(esquinaB[0],esquinaB[1],esquinaB[2]) );
	vertices2->push_back( osg::Vec3(esquinaC[0],esquinaC[1],esquinaC[2]) );
	vertices2->push_back( osg::Vec3(esquinaD[0],esquinaD[1],esquinaD[2]));


	// Declaro la normal
	osg::ref_ptr<osg::Vec3Array> normals2 = new osg::Vec3Array;
	normals2->push_back( osg::Vec3(zVecCor[0],zVecCor[1],zVecCor[2]) );

	// Colores en los vertices
	osg::ref_ptr<osg::Vec4Array> colors2 = new osg::Vec4Array;
	colors2->push_back( osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );
	//colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
	//colors->push_back( osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );
	//colors->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

	osg::ref_ptr<osg::Geometry> quad2 = new osg::Geometry;
	quad2->setVertexArray( vertices2.get() );
	quad2->setNormalArray( normals2.get() );
	quad2->setNormalBinding( osg::Geometry::BIND_OVERALL );
	quad2->setColorArray( colors2.get() );
	quad2->setColorBinding( osg::Geometry::BIND_OVERALL );

	quad2->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );
	osg::ref_ptr<osg::Geode> rootPlane2 = new osg::Geode;
	rootPlane2->addDrawable( quad2.get() );

	////////////////////////////////////////////////////




	// Lo agrego al Root principal de la Escena
	root->addChild(rootPlane);
	root->addChild(rootPlane2);


	// Creo Bounding Spheres de las Figuras en la scene. Esto es para las colisiones.
	// Los Bounding Spheres se esta colocando en las mismas posiciones y mismo radio que las esferas.
	osg::BoundingSphere bs_unitSphere = osg::BoundingSphere(osg::Vec3f(positionX_final,positionY_final,positionZ_final),0.3f);
	osg::BoundingSphere bs2_esferaEstatica = osg::BoundingSphere(osg::Vec3f(positionX_esferaEstatica,positionY_esferaEstatica,positionZ_esferaEstatica),2.0f);


	osg::ref_ptr<osg::LightSource> lightSource (new osg::LightSource);
	lightSource->getLight()->setPosition(osg::Vec4(3.0, 10.0, 3.0,1.0));
	lightSource->getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1));
	lightSource->getLight()->setDiffuse(osg::Vec4(0.8, 0.8, 0.8, 1));
	root->addChild(lightSource);


	// OSG1 viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);

	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	//viewer.getCamera()->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));
	///viewer.getCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	//viewer.getCamera()->setViewMatrix(osg::Matrix::identity());

	viewer.setSceneData( root );
	viewer.realize();

	myKeyboardEventHandler* myFirstEventHandler = new myKeyboardEventHandler();
	viewer.addEventHandler(myFirstEventHandler); 


	while( !viewer.done() )
	{

		if((puntoDireccionDrawable->getBound()).intersects(quad->getBound()))
		{
			choque=true;
			//printf("Colision");
			//puntoDireccionDrawable->setColor( osg::Vec4(255.0, 255.1, 0.1, 0.1) );
			//if(!toca)
			//{
			//	positionCaidaX=positionX;
			//	positionCaidaY=positionY;
			//	positionCaidaZ=positionZ;
			//}
			//toca=true;
		}
		else
			puntoDireccionDrawable->setColor( osg::Vec4(255.0, 0.0, 0.0, 0.1) );

		//positionZ = positionZ + 0.01f;
		//positionX = positionX - 0.01f;
		unitSphereXForm->setPosition(osg::Vec3(positionX,positionY,positionZ));
		puntoDireccionXForm->setPosition(osg::Vec3(positionX_final,positionY_final,positionZ_final));
		bs_unitSphere.set(osg::Vec3f(positionX,positionY,positionZ),0.3f);
		esferaEstaticaDrawable->setColor( osg::Vec4(0.1, 255.0, 0.1, 0.1) );
		viewer.frame();
	}
	return 0;
}

void iniciarOSG(){

	DWORD dwThreadId, dwThrdParam = 1;
	HANDLE hiloParaTracking;

	//Creacion del hilo para Tracking
	hiloParaTracking = CreateThread(
		NULL, // default security attributes
		0, // use default stack size
		iniciarOSG, // thread function
		&dwThrdParam, // argument to thread function
		0, // use default creation flags
		&dwThreadId); // returns the thread identifier
	if (hiloParaTracking == NULL)
		printf("CreateThread() failed, error: %d.\n", GetLastError());
	//else, gives some prompt...
	else
	{
		printf("Creacion del hilo de tracking exitosa\n");
		printf("The thread ID: %u.\n", dwThreadId);
	}
	if (CloseHandle(hiloParaTracking) != 0)
		printf("Handle to thread closed successfully.\n");




}

///////////////// CALIBRACION//////////////////////

void calibrar(float x,float y,float z){

	if (indexFlag == 0){
		std::cout << "----Proceso de Calibración-----\n\n Ingrese la primera coordenada" << std::endl;
		indexFlag++;
		/*// Test for allowing this type of comment through.
		gmtl::Matrix44f test_matrix;
		gmtl::identity(test_matrix);
		gmtl::invert(test_matrix);

		std::cout << test_matrix[0][0] << test_matrix[0][1] <<std::endl;


		gmtl::Vec3f test_vector;
		test_vector += gmtl::Vec3f(1.0, 0.0f, 1.0f);*/

	}


	if (_kbhit() && indexEsquina == 0) //Primera Esquina
	{
		_getch();
		esquina1.x = x;
		esquina1.y = y;
		esquina1.z = z;
		esquinaA = gmtl::Vec3f(x, y, z);
		indexEsquina++;

		std::cout << "Primera coordenada : " << " X:"<<esquina1.x<<" Y:"<<esquina1.y<<" Z:"<< esquina1.z << "\n\n Ingrese la segunda coordenada" << std::endl;

	}else if (_kbhit() && indexEsquina == 1) //Segunda Esquina
	{
		_getch();
		esquina2.x = x;
		esquina2.y = y;
		esquina2.z = z;
		esquinaB = gmtl::Vec3f(x, y, z);
		indexEsquina++;
		std::cout << "Segunda coordenada : " << " X:" << esquina2.x << " Y:" << esquina2.y << " Z:" << esquina2.z << "\n\n Ingrese la tercera coordenada" << std::endl;

	} else if (_kbhit() && indexEsquina == 2) //Tercera Esquina
	{
		_getch();
		esquina3.x = x;
		esquina3.y = y;
		esquina3.z = z;
		esquinaC = gmtl::Vec3f(x, y, z);
		indexEsquina++;
		std::cout << "Tercera coordenada : " << " X:" << esquina3.x << " Y:" << esquina3.y << " Z:" << esquina3.z << "\n\n Ingrese la cuarta coordenada" << std::endl;

	}else if (_kbhit() && indexEsquina == 3) //Cuarta Esquina
	{
		_getch();
		esquina4.x = x;
		esquina4.y = y;
		esquina4.z = z;
		esquinaD = gmtl::Vec3f(x, y, z);
		mCalibracion = true;

		std::cout << "Cuarta coordenada : " << " X:" << esquina4.x << " Y:" << esquina4.y << " Z:" << esquina4.z << std::endl;
		//calcular el sistema de coordenadas //esquinaA origen // zVecCor vector perpendicular al plano
		xVecCor=esquinaB-esquinaA;
		gmtl::normalize(xVecCor);
		std::cout<<esquinaA[1]<<" ;"<<esquinaB[1];
		gmtl::Vec3f tempYVecCor=esquinaD-esquinaA;
		gmtl::normalize(tempYVecCor);
		gmtl::cross(zVecCor,xVecCor,tempYVecCor);
		gmtl::normalize(zVecCor);
		gmtl::cross(yVecCor,zVecCor,xVecCor);
		gmtl::normalize(yVecCor);
		gmtl::cross(xVecCor,yVecCor,zVecCor);
		gmtl::normalize(xVecCor);
		
		Pluma::C[0][0]=xVecCor[0];
		Pluma::C[1][0]=xVecCor[1];
		Pluma::C[2][0]=xVecCor[2];
		Pluma::C[0][1]=yVecCor[0];
		Pluma::C[1][1]=yVecCor[1];
		Pluma::C[2][1]=yVecCor[2];
		Pluma::C[0][2]=zVecCor[0];
		Pluma::C[1][2]=zVecCor[1];
		Pluma::C[2][2]=zVecCor[2];

		Pluma::C_inv[0][0]=xVecCor[0];
		Pluma::C_inv[1][0]=xVecCor[1];
		Pluma::C_inv[2][0]=xVecCor[2];
		Pluma::C_inv[0][1]=yVecCor[0];
		Pluma::C_inv[1][1]=yVecCor[1];
		Pluma::C_inv[2][1]=yVecCor[2];
		Pluma::C_inv[0][2]=zVecCor[0];
		Pluma::C_inv[1][2]=zVecCor[1];
		Pluma::C_inv[2][2]=zVecCor[2];

		gmtl::invert(Pluma::C_inv);
		//std::cout << "n: "<<zVecCor[0]<<" ;"<<zVecCor[1]<<"; "<<zVecCor[2]<<std::endl;
		
		//Primer D
		Pluma::D=zVecCor[0]*esquinaA[0]+zVecCor[1]*esquinaA[1]+zVecCor[2]*esquinaA[02]; //D de la ecuacion del plano
		



		// The 4 points that select quadilateral on the input , from top-left in clockwise order
		// These four pts are the sides of the rect box used as input 
		gmtl::Vec3f tempA=gmtl::Vec3f (0,0,0);
		coordenadasLocalesA=Pluma::C_inv*tempA;	
		gmtl::Vec3f tempB=esquinaB-esquinaA;//gmtl::Vec3f (esquinaB[0]-esquinaA[0],esquinaB[1]-esquinaA[1],esquinaB[2]-esquinaA[2]);
		coordenadasLocalesB=Pluma::C_inv*tempB;	
		gmtl::Vec3f tempC=esquinaC-esquinaA;//gmtl::Vec3f (esquinaC[0]-esquinaA[0],esquinaC[1]-esquinaA[1],esquinaC[2]-esquinaA[2]);
		coordenadasLocalesC=Pluma::C_inv*tempC;	
		gmtl::Vec3f tempD=esquinaD-esquinaA;//gmtl::Vec3f (esquinaD[0]-esquinaA[0],esquinaD[1]-esquinaA[1],esquinaD[2]-esquinaA[2]);
		coordenadasLocalesD=Pluma::C_inv*tempD;	

		gmtl::Vec3f xlocal,zlocal,ylocal;
		xlocal=coordenadasLocalesB-coordenadasLocalesA;
		gmtl::normalize(xlocal);		
		gmtl::Vec3f tempYLoc=coordenadasLocalesD-coordenadasLocalesA;
		gmtl::normalize(tempYLoc);
		gmtl::cross(zlocal,xlocal,tempYLoc);
		gmtl::normalize(zlocal);
		gmtl::cross(ylocal,zlocal,xlocal);
		gmtl::normalize(ylocal);
		Pluma::C_alt[0][0]=xlocal[0];
		Pluma::C_alt[1][0]=xlocal[1];
		Pluma::C_alt[2][0]=xlocal[2];
		Pluma::C_alt[0][1]=ylocal[0];
		Pluma::C_alt[1][1]=ylocal[1];
		Pluma::C_alt[2][1]=ylocal[2];
		Pluma::C_alt[0][2]=zlocal[0];
		Pluma::C_alt[1][2]=zlocal[1];
		Pluma::C_alt[2][2]=zlocal[2];
		gmtl::invert(Pluma::C_alt);

		

		//Segundo D
		Pluma::D=zlocal[0]*coordenadasLocalesB[0]+zlocal[1]*coordenadasLocalesB[1]+zlocal[2]*coordenadasLocalesB[2]; //D de la ecuacion del plano
		Pluma::divisor=sqrtf(zlocal[0]*zlocal[0]+zlocal[1]*zlocal[1]+zlocal[2]*zlocal[2]);

		inputQuad[3] = Point2f( coordenadasLocalesD[0],coordenadasLocalesD[1]);
		inputQuad[2] = Point2f( coordenadasLocalesC[0],coordenadasLocalesC[1]);
		inputQuad[1] = Point2f( coordenadasLocalesB[0],coordenadasLocalesB[1]);
		inputQuad[0] = Point2f( coordenadasLocalesA[0],coordenadasLocalesA[1]);  


		// The 4 points where the mapping is to be done , from top-left in clockwise order
		outputQuad[0] = Point2f( 0,0 );
		outputQuad[1] = Point2f(Pluma::WIDTH,0 );
		outputQuad[2] = Point2f( Pluma::WIDTH,Pluma::HEIGHT);
		outputQuad[3] = Point2f( 0,Pluma::HEIGHT );
	//	std::cout<<lambda <<std::endl;

		// Get the Perspective Transform Matrix ie lambda 
		Pluma::lambda = cv::getPerspectiveTransform( inputQuad, outputQuad );
		
		Pluma::xlocal=xlocal;
		Pluma::ylocal=ylocal;
		Pluma::zlocal=zlocal;

		Pluma::esquinaA=esquinaA;
		Pluma::esquinaB=esquinaB;
		Pluma::esquinaC=esquinaC;
		Pluma::esquinaD=esquinaD;



		iniciarOSG();

	}
}

void calcularOrientacion(gmtl::Vec3f m1,gmtl::Vec3f m2, gmtl::Vec3f m3, gmtl::Vec3f posicion){
	gmtl::Vec3f vector1,vector2,dir;
	vector1 = m1-m2;
	vector2=m3-m2;
	gmtl::cross(dir,vector2,vector1);
	float posicionX=posicion[0];
	float posicionY=posicion[1];
	float posicionZ=posicion[2];
	float x1= posicionX-(dir[0]);
	float y1= posicionY-(dir[1]);
	float z1=posicionZ-(dir[2]);

	float a=-1.0*dir[0];//x1-posicionX; // vector direccion
	float b=-1.0*dir[1];//y1-posicionY;
	float c=-1.0*dir[2];//z1-posicionZ;
	// printf("Tracker Position:(%.4f,%.4f,%.4f) Orientation:(%.2f,%.2f,%.2f,%.2f) yInter:%.2f \n",
	//  x, y, z,
	// qx, qy, qz, qw,b);
	//x0 posicionX y0 posicionY z0 posicionZ
	/*if(b!=0.0){
	float t=-posicionY/b;
	positionX_final=posicionX+t*a;
	positionY_final=0.0;
	positionZ_final=posicionZ+t*c;


	}*/
//	float divisor=zVecCor[0]*a+zVecCor[1]*b+zVecCor[2]*c;
	//float dividendo=D-(zVecCor[0]*posicionX+zVecCor[1]*posicionY+zVecCor[2]*posicionZ);
	if(true){//divisor!=0){
		//std::cout<<"Insterseccion"<<std::endl;
		/*float t=dividendo/divisor;
		positionX_final=posicionX+t*a;
		positionY_final=posicionY+t*b;
		positionZ_final=posicionZ+t*c;*/

		/// Sacar posicion punta de la pluma
		float largo_pluma = 0.135*10.0f;
		float k = sqrtf((float)(largo_pluma*largo_pluma) / (float)(a*a + b*b + c*c));
		float h = positionY + k*b;
		positionX_final = positionX + k*a;
		positionY_final = positionY + k*b;
		positionZ_final = positionZ + k*c;

		//gmtl::Vec3f temp=gmtl::Vec3f (positionX_final-esquinaA[0],positionY_final-esquinaA[1],positionZ_final-esquinaA[2]);
		gmtl::Vec3f temp=gmtl::Vec3f (positionX_final,positionY_final,positionZ_final);
		temp=temp-esquinaA;
	//	gmtl::Vec3f temp=gmtl::Vec3f (posicionX-esquinaA[0],posicionY-esquinaA[1],posicionZ-esquinaA[2]);
	coordenadasLocales=Pluma::C_inv*temp;
	//gmtl::Vec3f coordenadasLocales_alt=C_alt*coordenadasLocales;
		//std::cout<<"Coordenadas Golbal"<<positionX_final<<";"<<positionY_final<<";"<<positionZ_final<<std::endl;
	
	//std::cout<<"Coordenadas plano local"<<coordenadasLocales[0]<<";"<<coordenadasLocales[1]<<";"<<coordenadasLocales[2]<<std::endl;
//std::cout<<"Coordenadas plano local"<<coordenadasLocales_alt[0]<<";"<<coordenadasLocales_alt[1]<<";"<<coordenadasLocales_alt[2]<<std::endl;
	
	float d=(coordenadasLocales[0]*Pluma::zlocal[0]+coordenadasLocales[1]*Pluma::zlocal[1]+coordenadasLocales[2]*Pluma::zlocal[2]+Pluma::D)/Pluma::divisor	;
	// Apply the Perspective Transform just found to the src image
	//std::cout<<"zlocal:"<<zlocal<<std::endl;
	//std::cout<<"D:"<<D<<std::endl;
	std::cout<<"Distancia:"<<d<<std::endl;
		std::vector<Point2f> punto_original(1);
		std::vector<Point2f> punto_transformado(1);
		Point2f seed=Point2f(coordenadasLocales[0],coordenadasLocales[1]);
		punto_original[0] = seed;
		cv::perspectiveTransform( punto_original,  punto_transformado, Pluma::lambda);

		Point2f tuio=punto_transformado[0];
		float xx=tuio.x/Pluma::WIDTH;
		float yy=(float)(tuio.y/Pluma::HEIGHT);
		//std::cout<<"Coordenadas plano local: X:"<<xx<<" Y: "<<yy<<std::endl;
		TuioCursor*cursor_1 = tuioServer->getTuioCursor(idCursor1);
		//std::cout<<"Altura: "<<coordenadasLocales[2]<<std::endl;
		if(d<0.09){
			currentTime = TuioTime::getSessionTime();
			tuioServer->initFrame(currentTime);
			
					if(numMuestra%3==0)
						muestra1=gmtl::Vec3f(xx,1-yy,0.0);
					else if(numMuestra%3==1)
						muestra2=gmtl::Vec3f(xx,1-yy,0.0);
					else if(numMuestra%3==2)
						muestra3=gmtl::Vec3f(xx,1-yy,0.0);
					
					numMuestra++;

					if(numMuestra<3){
						promedio=gmtl::Vec3f(xx,1-yy,0.0);
					}else{
						promedio=gmtl::Vec3f((muestra1[0]+muestra2[0]+muestra3[0])/3.0f,(muestra1[1]+muestra2[1]+muestra3[1])/3.0f,0.0);
					}

			if (cursor == NULL || idCursor1 == -1000)
			{

				cursor = tuioServer->addTuioCursor(promedio[0], promedio[1],1);
				idCursor1 = cursor->getCursorID();
			}
			else{
				tuioServer->updateTuioCursor(cursor,promedio[0], promedio[1]);
				printf("TUIO: %.2f,%.2f SESSION:%d\n", cursor->getX(), cursor->getY(),cursor->getSessionID());
				/*printf("Esquina1: %.2f,%.2f ,%.2f ,%.2f \n", esquina1.x, esquina1.y, esquina1.z );
				printf("Esquina2: %.2f,%.2f ,%.2f ,%.2f \n", esquina2.x, esquina2.y, esquina2.z);
				printf("Esquina3: %.2f,%.2f ,%.2f ,%.2f \n", esquina3.x, esquina3.y, esquina3.z);
				printf("Esquina4: %.2f,%.2f ,%.2f ,%.2f \n", esquina4.x, esquina4.y, esquina4.z);*/
			}
			tuioServer->commitFrame();
		}else{
			//printf("numero de cursores: %d", ncursors);
			if (cursor != NULL){
				currentTime = TuioTime::getSessionTime();
				tuioServer->initFrame(currentTime);
				tuioServer->removeTuioCursor(cursor);
				tuioServer->commitFrame();
				idCursor1 = -1000;
				cursor = NULL;
				numMuestra=0;
				printf("Eliminado");
			}
		}


	}


}
void calcularOrientacion2(gmtl::Vec3f m1,gmtl::Vec3f m2, gmtl::Vec3f m3, gmtl::Vec3f posicion){
	gmtl::Vec3f vector1,vector2,dir;
	vector1 = m1-m2;
	vector2=m3-m2;
	gmtl::cross(dir,vector1,vector2);
	float posicionX=posicion[0];
	float posicionY=posicion[1];
	float posicionZ=posicion[2];
	float x1= posicionX-(dir[0]);
	float y1= posicionY-(dir[1]);
	float z1=posicionZ-(dir[2]);

	float a=-1.0*dir[0];//x1-posicionX; // vector direccion
	float b=-1.0*dir[1];//y1-posicionY;
	float c=-1.0*dir[2];//z1-posicionZ;
	// printf("Tracker Position:(%.4f,%.4f,%.4f) Orientation:(%.2f,%.2f,%.2f,%.2f) yInter:%.2f \n",
	//  x, y, z,
	// qx, qy, qz, qw,b);
	//x0 posicionX y0 posicionY z0 posicionZ
	/*if(b!=0.0){
	float t=-posicionY/b;
	positionX_final=posicionX+t*a;
	positionY_final=0.0;
	positionZ_final=posicionZ+t*c;


	}*/
//	float divisor=zVecCor[0]*a+zVecCor[1]*b+zVecCor[2]*c;
	//float dividendo=D-(zVecCor[0]*posicionX+zVecCor[1]*posicionY+zVecCor[2]*posicionZ);
	if(true){//divisor!=0){
		//std::cout<<"Insterseccion"<<std::endl;
		/*float t=dividendo/divisor;
		positionX_final=posicionX+t*a;
		positionY_final=posicionY+t*b;
		positionZ_final=posicionZ+t*c;*/

		/// Sacar posicion punta de la pluma
		float largo_pluma = 0.135*10.0f;
		float k = sqrtf((float)(largo_pluma*largo_pluma) / (float)(a*a + b*b + c*c));
		float h = positionY + k*b;
		float positionX_final2 = positionX + k*a;
		float positionY_final2 = positionY + k*b;
		float positionZ_final2 = positionZ + k*c;

		//gmtl::Vec3f temp=gmtl::Vec3f (positionX_final-esquinaA[0],positionY_final-esquinaA[1],positionZ_final-esquinaA[2]);
		gmtl::Vec3f temp=gmtl::Vec3f (positionX_final2,positionY_final2,positionZ_final2);
		temp=temp-esquinaA;
	//	gmtl::Vec3f temp=gmtl::Vec3f (posicionX-esquinaA[0],posicionY-esquinaA[1],posicionZ-esquinaA[2]);
	coordenadasLocales2=Pluma::C_inv*temp;
	//gmtl::Vec3f coordenadasLocales_alt=C_alt*coordenadasLocales;
		//std::cout<<"Coordenadas Golbal"<<positionX_final<<";"<<positionY_final<<";"<<positionZ_final<<std::endl;
	
	//std::cout<<"Coordenadas plano local"<<coordenadasLocales[0]<<";"<<coordenadasLocales[1]<<";"<<coordenadasLocales[2]<<std::endl;
//std::cout<<"Coordenadas plano local"<<coordenadasLocales_alt[0]<<";"<<coordenadasLocales_alt[1]<<";"<<coordenadasLocales_alt[2]<<std::endl;
	
	float d=(coordenadasLocales2[0]*Pluma::zlocal[0]+coordenadasLocales2[1]*Pluma::zlocal[1]+coordenadasLocales2[2]*Pluma::zlocal[2]+Pluma::D)/Pluma::divisor;
	// Apply the Perspective Transform just found to the src image
	//std::cout<<"zlocal:"<<zlocal<<std::endl;
	//std::cout<<"D:"<<D<<std::endl;
	std::cout<<"Distancia2:"<<d<<std::endl;
		std::vector<Point2f> punto_original(1);
		std::vector<Point2f> punto_transformado(1);
		Point2f seed=Point2f(coordenadasLocales2[0],coordenadasLocales2[1]);
		punto_original[0] = seed;
		cv::perspectiveTransform( punto_original,  punto_transformado, Pluma::lambda);

		Point2f tuio=punto_transformado[0];
		float xx=tuio.x/Pluma::WIDTH;
		float yy=(float)(tuio.y/Pluma::HEIGHT);
		//std::cout<<"Coordenadas plano local: X:"<<xx<<" Y: "<<yy<<std::endl;
		//TuioCursor*cursor_1 = tuioServer->getTuioCursor(idCursor1);
		//std::cout<<"Altura: "<<coordenadasLocales[2]<<std::endl;
		if(d<0.09){
			currentTime = TuioTime::getSessionTime();
			tuioServer->initFrame(currentTime);
			
					if(numMuestra2%3==0)
						muestra21=gmtl::Vec3f(xx,1-yy,0.0);
					else if(numMuestra2%3==1)
						muestra22=gmtl::Vec3f(xx,1-yy,0.0);
					else if(numMuestra2%3==2)
						muestra23=gmtl::Vec3f(xx,1-yy,0.0);
					
					numMuestra2++;

					if(numMuestra2<3){
						promedio2=gmtl::Vec3f(xx,1-yy,0.0);
					}else{
						promedio2=gmtl::Vec3f((muestra21[0]+muestra22[0]+muestra23[0])/3.0f,(muestra21[1]+muestra22[1]+muestra23[1])/3.0f,0.0);
					}

			if (cursor2 == NULL || idCursor2 == -1000)
			{

				cursor2 = tuioServer->addTuioCursor(promedio2[0], promedio2[1],2);
				idCursor2 = cursor2->getCursorID();
			}
			else{
				tuioServer->updateTuioCursor(cursor2,promedio2[0], promedio2[1]);
				printf("TUIO2: %.2f,%.2f SESSION:%d \n", cursor2->getX(), cursor2->getY(),cursor2->getSessionID());
				/*printf("Esquina1: %.2f,%.2f ,%.2f ,%.2f \n", esquina1.x, esquina1.y, esquina1.z );
				printf("Esquina2: %.2f,%.2f ,%.2f ,%.2f \n", esquina2.x, esquina2.y, esquina2.z);
				printf("Esquina3: %.2f,%.2f ,%.2f ,%.2f \n", esquina3.x, esquina3.y, esquina3.z);
				printf("Esquina4: %.2f,%.2f ,%.2f ,%.2f \n", esquina4.x, esquina4.y, esquina4.z);*/
			}
			tuioServer->commitFrame();
		}else{
			//printf("numero de cursores: %d", ncursors);
			if (cursor2 != NULL){
				currentTime = TuioTime::getSessionTime();
				tuioServer->initFrame(currentTime);
				tuioServer->removeTuioCursor(cursor2);
				tuioServer->commitFrame();
				idCursor2 = -1000;
				cursor2 = NULL;
				numMuestra2=0;
				printf("Eliminado");
			}
		}


	}


}

// Main application 
int main( int argc, char* argv[] )
{

	tuioServer = new TuioServer();
	printf("== NaturalPoint Tracking Tools API Marker Sample =======---\n");
	printf("== (C) NaturalPoint, Inc.\n\n");

	printf("Initializing NaturalPoint Devices\n");
	TT_Initialize();

	// Do an update to pick up any recently-arrived cameras.
	TT_Update();

	// Load a project file from the executable directory.
	printf( "Loading Project: test2.ttp\n\n" );
	CheckResult( TT_LoadProject("demo.ttp") );

	// List all detected cameras.
	printf( "Cameras:\n" );
	for( int i = 0; i < TT_CameraCount(); i++)
	{
		printf( "\t%s\n", TT_CameraName(i) );
	}
	printf("\n");

	// List all defined rigid bodies.
	printf("Rigid Bodies:\n");
	Pluma  *listaPlumas[15];
	for( int i = 0; i < TT_TrackableCount(); i++)
	{
		printf("\t%s\n", TT_TrackableName(i));
		listaPlumas[i]=new Pluma(i+1);
	}
	printf("\n");

	int frameCounter = 0;

	// Poll API data until the user hits a keyboard key.
	while( true)//!_kbhit() )
	{
		if( TT_Update() == NPRESULT_SUCCESS )
		{
			frameCounter++;

			// Update tracking information every 100 frames (for example purposes).
			if( true)//(frameCounter%2) == 0 )
			{
				float   yaw,pitch,roll;
				float   x,y,z;
				float   qx,qy,qz,qw;
				bool    tracked;

			//	printf( "Frame #%d: (Markers: %d)\n", frameCounter, TT_FrameMarkerCount() );

				for( int i = 0; i < TT_TrackableCount(); i++ )
				{
					TT_TrackableLocation( i, &x,&y,&z, &qx,&qy,&qz,&qw, &yaw,&pitch,&roll );

					if( TT_IsTrackableTracked( i ) )
					{
						if(!mCalibracion)
						printf( "\t%s: Pos (%.3f, %.3f, %.3f) Orient (%.1f, %.1f, %.1f)\n", TT_TrackableName( i ),
							x*10.0f,z*10.0f,y*10.0f, yaw, pitch, roll );
						/////////////////////////////////////////////////////////////////////////

						positionX=x*10.0f;
						positionY=z*10.0f;
						positionZ=y*10.0f;

						/*	dirx= 2*(qx * qy - qz*qw);
						diry = 1-2*(qx*qx+qz*qz);
						dirz= 2*(qy*qz+qx*qw);

						r=1;
						float x1= positionX-(dirx/r);
						float y1= positionY-(diry/r);
						float z1=positionZ-(dirz/r);

						float a=-x1+positionX; // vector direccion
						float b=-y1+positionY;
						float c=-z1+positionZ;
						printf("Tracker Position:(%.4f,%.4f,%.4f) Orientation:(%.2f,%.2f,%.2f,%.2f) yInter:%.2f \n",
						x, y, z,
						qx, qy, qz, qw,b);
						if(b!=0.0){
						float t=-positionY/b;
						positionX_final=positionX+t*a;
						positionY_final=0.0;
						positionZ_final=positionZ+t*c;

						/// Sacar posicion punta de la pluma
						float largo_pluma = 0.135*10.0f;
						float k = sqrt((largo_pluma*largo_pluma) / (a*a + b*b + c*c));
						float h = positionY + k*b;
						positionX_final=positionX+k*a;
						positionY_final=positionY + k*b;
						positionZ_final=positionZ+k*c;
						printf("Punta Position:(%.4f,%.4f,%.4f) \n",
						positionX_final,positionY_final,positionZ_final);


						}*/
						//positionX_final=x*10.0f;
						//positionY_final=y*10.0f;
						//positionZ_final=z*-10.0f;
						/////////////////////////////////////////////////////////////////////////



						TransformMatrix xRot( TransformMatrix::RotateX( -roll * kRadToDeg ) );
						TransformMatrix yRot( TransformMatrix::RotateY( -yaw * kRadToDeg ) );
						TransformMatrix zRot( TransformMatrix::RotateZ( -pitch * kRadToDeg ) );

						// Compose the local-to-world rotation matrix in XZY (roll, pitch, yaw) order.
						TransformMatrix worldTransform = xRot * zRot * yRot;

						// Inject world-space coordinates of the origin.
						worldTransform.SetTranslation( x, y, z );

						// Invert the transform matrix to convert from a local-to-world to a world-to-local.
						worldTransform.Invert();

						float   mx, my, mz;
						gmtl::Vec3f m1,m2,m3,posicion; /// 
						int     markerCount = TT_TrackableMarkerCount( i );
						for( int j = 0; j < markerCount; ++j )
						{
							// Get the world-space coordinates of each rigid body marker.
							TT_TrackablePointCloudMarker( i, j, tracked, mx, my, mz );
							if(j==0){
								m1=gmtl::Vec3f(mx,mz,my);
							}else if(j==1){
								m2=gmtl::Vec3f(mx,mz,my);
							}
							else{
								m3=gmtl::Vec3f(mx,mz,my);
							}
							// Transform the rigid body point from world coordinates to local rigid body coordinates.
							// Any world-space point can be substituted here to transform it into the local space of
							// the rigid body.
							Point4  worldPnt( mx, my, mz, 1.0f );
							Point4  localPnt = worldTransform * worldPnt;

							//printf( "\t\t%d: World (%.3f, %.3f, %.3f) Local (%.3f, %.3f, %.3f)\n", j + 1, 
								//mx, my, mz, localPnt[0], localPnt[1], localPnt[2] );
						}
						/*if(!mCalibracion&&i==0){
							calibrar(positionX,positionY,positionZ);
						}else if(mCalibracion&&i==0){

							posicion=gmtl::Vec3f(positionX,positionY,positionZ);
							//calcularOrientacion(m1,m2,m3,posicion);}
							listaPlumas[i]->calcularOrientacion(positionX,positionY,positionZ,m1,m2,m3,posicion);}
						else if(mCalibracion&&i==1){
							posicion=gmtl::Vec3f(positionX,positionY,positionZ);
							//calcularOrientacion2(m1,m2,m3,posicion);
							listaPlumas[i]->calcularOrientacion(positionX,positionY,positionZ,m1,m2,m3,posicion);
						}
						else if(mCalibracion&&i==2){
							posicion=gmtl::Vec3f(positionX,positionY,positionZ);
							//calcularOrientacion2(m1,m2,m3,posicion);
							listaPlumas[i]->calcularOrientacion(positionX,positionY,positionZ,m1,m2,m3,posicion);
						}*/
						if(!mCalibracion){
								calibrar(positionX,positionY,positionZ);
						}else{
							listaPlumas[i]->calcularOrientacion(positionX,positionY,positionZ,m1,m2,m3,posicion);
						}

					}
					else
					{
						printf( "\t%s: Not Tracked\n", TT_TrackableName( i ) );
					}
				}
			}
		}
		Sleep(2);
	}

	printf( "Shutting down NaturalPoint Tracking Tools\n" );
	CheckResult( TT_Shutdown() );

	printf( "Complete\n" );
	while( !_kbhit() )
	{
		Sleep(20);
	}

	TT_FinalCleanup();
	getchar();
	return 0;


}


void CheckResult( NPRESULT result )   //== CheckResult function will display errors and ---
	//== exit application after a key is pressed =====---
{
	if( result!= NPRESULT_SUCCESS)
	{
		// Treat all errors as failure conditions.
		printf( "Error: %s\n\n(Press any key to continue)\n", TT_GetResultString(result) );

		Sleep(20);
		getchar();
		exit(1);
	}
}

//
// Point4
//

Point4::Point4( float x, float y, float z, float w )
{
	mData[0] = x;
	mData[1] = y;
	mData[2] = z;
	mData[3] = w;
}

//
// TransformMatrix
//

TransformMatrix::TransformMatrix()
{
	for( int i = 0; i < 4; ++i )
	{
		for( int j = 0; j < 4; ++j )
		{
			if( i == j )
			{
				mData[i][j] = 1.0f;
			}
			else
			{
				mData[i][j] = 0.0f;
			}
		}
	}
}

TransformMatrix::TransformMatrix( float m11, float m12, float m13, float m14,
	float m21, float m22, float m23, float m24,
	float m31, float m32, float m33, float m34,
	float m41, float m42, float m43, float m44 )
{
	mData[0][0] = m11;
	mData[0][1] = m12;
	mData[0][2] = m13;
	mData[0][3] = m14;
	mData[1][0] = m21;
	mData[1][1] = m22;
	mData[1][2] = m23;
	mData[1][3] = m24;
	mData[2][0] = m31;
	mData[2][1] = m32;
	mData[2][2] = m33;
	mData[2][3] = m34;
	mData[3][0] = m41;
	mData[3][1] = m42;
	mData[3][2] = m43;
	mData[3][3] = m44;
}

void TransformMatrix::SetTranslation( float x, float y, float z )
{
	mData[0][3] = x;
	mData[1][3] = y;
	mData[2][3] = z;
}

void TransformMatrix::Invert()
{
	// Exploit the fact that we are dealing with a rotation matrix + translation component.
	// http://stackoverflow.com/questions/2624422/efficient-4x4-matrix-inverse-affine-transform

	float   tmp;
	float   vals[3];

	// Transpose left-upper 3x3 (rotation) sub-matrix
	tmp = mData[0][1]; mData[0][1] = mData[1][0]; mData[1][0] = tmp;
	tmp = mData[0][2]; mData[0][2] = mData[2][0]; mData[2][0] = tmp;
	tmp = mData[1][2]; mData[1][2] = mData[2][1]; mData[2][1] = tmp;

	// Multiply translation component (last column) by negative inverse of upper-left 3x3.
	for( int i = 0; i < 3; ++i )
	{
		vals[i] = 0.0f;
		for( int j = 0; j < 3; ++j )
		{
			vals[i] += -mData[i][j] * mData[j][3];
		}
	}
	for( int i = 0; i < 3; ++i )
	{
		mData[i][3] = vals[i];
	}
}

TransformMatrix TransformMatrix::RotateX( float rads )
{
	return TransformMatrix( 1.0, 0.0, 0.0, 0.0,
		0.0, cos( rads ), -sin( rads ), 0.0,
		0.0, sin( rads ), cos( rads ), 0.0,
		0.0, 0.0, 0.0, 1.0 );
}

TransformMatrix TransformMatrix::RotateY( float rads )
{
	return TransformMatrix( cos( rads ), 0.0, sin( rads ), 0.0,
		0.0, 1.0, 0.0, 0.0,
		-sin( rads ), 0.0, cos( rads ), 0.0,
		0.0, 0.0, 0.0, 1.0 );
}

TransformMatrix TransformMatrix::RotateZ( float rads )
{
	return TransformMatrix( cos( rads ), -sin( rads ), 0.0, 0.0,
		sin( rads ), cos( rads ), 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0 );
}

TransformMatrix TransformMatrix::operator*( const TransformMatrix &rhs )
{
	TransformMatrix result;

	for( int i = 0; i < 4; ++i )
	{
		for( int j = 0; j < 4; ++j )
		{
			float rowCol = 0.0;
			for( int k = 0; k < 4; ++k )
			{
				rowCol += mData[i][k] * rhs.mData[k][j];
			}
			result.mData[i][j] = rowCol;
		}
	}
	return result;
}

Point4 TransformMatrix::operator*( const Point4 &v )
{
	const float *pnt = v.Data();
	float   result[4];

	for( int i = 0; i < 4; ++i )
	{
		float rowCol = 0.0;
		for( int k = 0; k < 4; ++k )
		{
			rowCol += mData[i][k] * pnt[k];
		}
		result[i] = rowCol;
	}
	return Point4( result[0], result[1], result[2], result[3] );
}


