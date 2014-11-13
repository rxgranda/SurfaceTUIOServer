#include "Pluma.h"
using namespace cti;
cv::Mat Pluma::lambda= cv::Mat::zeros(3, 3, CV_32FC1);
 gmtl::Vec3f Pluma::xlocal;
 gmtl::Vec3f Pluma::ylocal;
 gmtl::Vec3f Pluma::zlocal;
 gmtl::Vec3f Pluma::esquinaA;
 gmtl::Vec3f Pluma::esquinaB;
 gmtl::Vec3f Pluma::esquinaC;
 gmtl::Vec3f Pluma::esquinaD;
float Pluma::D=0;
float Pluma::divisor=1;
float Pluma::WIDTH=7680;
float Pluma::HEIGHT=4320;
float Pluma::ALTURA_PLUMA=0.135;


gmtl::Matrix33f Pluma::C_inv;
gmtl::Matrix33f Pluma::C;
gmtl::Matrix33f Pluma::C_alt;
TUIO::TuioServer *Pluma::tuioServer = new TUIO::TuioServer();

Pluma::Pluma(int numero)
{
	numeroPluma=numero;
	idCursor=-1000;
	cursor=NULL;
}


Pluma::~Pluma(void)
{
}


void Pluma::calcularOrientacion(float oPosX,float oPosY,float oPosZ,gmtl::Vec3f m1,gmtl::Vec3f m2, gmtl::Vec3f m3, gmtl::Vec3f posicion){
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
		float largo_pluma = ALTURA_PLUMA*10.0f;
		float k = sqrtf((float)(largo_pluma*largo_pluma) / (float)(a*a + b*b + c*c));
		float h = oPosY + k*b;
		float positionX_final = oPosX + k*a;
		float positionY_final = oPosY + k*b;
		float positionZ_final = oPosZ + k*c;

		//gmtl::Vec3f temp=gmtl::Vec3f (positionX_final-esquinaA[0],positionY_final-esquinaA[1],positionZ_final-esquinaA[2]);
		gmtl::Vec3f temp=gmtl::Vec3f (positionX_final,positionY_final,positionZ_final);
		temp=temp-esquinaA;
	//	gmtl::Vec3f temp=gmtl::Vec3f (posicionX-esquinaA[0],posicionY-esquinaA[1],posicionZ-esquinaA[2]);
		coordenadasLocales=Pluma::C_inv*temp;
	//gmtl::Vec3f coordenadasLocales_alt=C_alt*coordenadasLocales;
		//std::cout<<"Coordenadas Golbal"<<positionX_final<<";"<<positionY_final<<";"<<positionZ_final<<std::endl;
	
	//std::cout<<"Coordenadas plano local"<<coordenadasLocales[0]<<";"<<coordenadasLocales[1]<<";"<<coordenadasLocales[2]<<std::endl;
//std::cout<<"Coordenadas plano local"<<coordenadasLocales_alt[0]<<";"<<coordenadasLocales_alt[1]<<";"<<coordenadasLocales_alt[2]<<std::endl;
	
	float d=(coordenadasLocales[0]*zlocal[0]+coordenadasLocales[1]*zlocal[1]+coordenadasLocales[2]*zlocal[2]+Pluma::D)/divisor	;
	// Apply the Perspective Transform just found to the src image
	//std::cout<<"zlocal:"<<zlocal<<std::endl;
	//std::cout<<"D:"<<D<<std::endl;
	std::cout<<"Distancia:"<<d<<std::endl;
	
		std::vector<cv::Point2f> punto_original(1);
		std::vector<cv::Point2f> punto_transformado(1);
		cv::Point2f seed=cv::Point2f(coordenadasLocales[0],coordenadasLocales[1]);
		punto_original[0] = seed;
		cv::perspectiveTransform( punto_original,  punto_transformado, Pluma::lambda);

		cv::Point2f tuio=punto_transformado[0];
		float xx=tuio.x/WIDTH;
		float yy=(float)(tuio.y/HEIGHT);
		//std::cout<<"Coordenadas plano local: X:"<<xx<<" Y: "<<yy<<std::endl;
		//TUIO::TuioCursor*cursor_1 = tuioServer->getTuioCursor(idCursor);
		//std::cout<<"Altura: "<<coordenadasLocales[2]<<std::endl;
		if(d<0.09){
			
			tuioServer->initFrame(TUIO::TuioTime::getSessionTime());
			
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

			if (cursor == NULL || idCursor == -1000)
			{
				
				cursor = tuioServer->addTuioCursor(promedio[0], promedio[1],numeroPluma);
				idCursor = cursor->getCursorID();
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
				
				tuioServer->initFrame(TUIO::TuioTime::getSessionTime());
				tuioServer->removeTuioCursor(cursor);
				tuioServer->commitFrame();
				idCursor = -1000;
				cursor = NULL;
				numMuestra=0;
				
			}
		}


	}


}