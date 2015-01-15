#pragma once
#include <gmtl/gmtl.h>
#include <gmtl/Matrix.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "TuioCursor.h"
#include "TuioServer.h"
namespace cti{
	class Pluma
	{
		public:
			bool invertir;
			int numeroPluma;
			int numMuestra;
			gmtl::Vec3f promedio, muestra1,muestra2,muestra3;
			TUIO::TuioCursor *cursor;
			long idCursor;
			gmtl::Vec3f coordenadasLocales;
	
			static gmtl::Matrix33f C_inv,C,C_alt;			
			static cv::Mat lambda;
			static TUIO::TuioServer *tuioServer;
			// escalares Para la ecuacion del plano
			static float D;
			static float divisor;
			static float WIDTH;
			static float HEIGHT;		
			static gmtl::Vec3f xlocal,zlocal,ylocal;
			static gmtl::Vec3f esquinaA, esquinaB, esquinaC, esquinaD;
			static float ALTURA_PLUMA;
			// escalares Para la ecuacion del plano
			void calcularOrientacion(float oPosX,float oPosY,float oPosZ,gmtl::Vec3f m1,gmtl::Vec3f m2, gmtl::Vec3f m3, gmtl::Vec3f posicion);
			Pluma(int numero, bool inversion=false);
			~Pluma(void);
	};
}

