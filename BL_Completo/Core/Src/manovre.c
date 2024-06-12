/*
 * manovre.c
 *
 *  Created on: Feb 19, 2024
 *      Author: angel
 */

#include <manovre.h>
#include "math.h"
#include "Configuration.h"


/*
 *  dy: spostamento lungo l'asse y (tra le corsie)
 *  lambda: lunghezza della traiettoria, approssimabile con lo spostamento totale lungo l' asse x
 *  v: velocità costante a cui si esegue la manovra
 *
 *  SORPASSO A SINISTRA
 *  	v>0, dy>0
 *  SORPASSO A DESTRA
 *  	v>0, dy<0
 *  PARCHEGGIO A SINISTRA
 *  	v<0, dy<0
 *  PARCHEGGIO A DESTRA
 *  	v<0, dy>0
 */

float calcolo_yaw_rate(float dy, float dx, float v, float t){
	float lambda = dx;


	if(dx < 2.00){
		//Correzione dulla dy (funzione lineare trovata sperimentalmente)
		dy += -0.04*dx + 0.08;

		//Approssimazione di lambda utilizzando il teorema di pitagora
		lambda = CORREZIONE_LAMBDA*sqrt(dx*dx + dy*dy);
	}
	//Caso limite, per non rompere l'andamento
	if(lambda*lambda <= 4 * dy*dy){
		lambda = 2 * dy;
	}


	return (2 * dy * v * PI * cos(PI / 2 - (2 * PI * t * v) / lambda)) /
			(lambda * lambda * sqrt(1 - pow((dy * v / lambda - (dy * v * sin(PI / 2 - (2 * t * v * PI) / lambda)) / lambda), 2) / (v * v)));
}
