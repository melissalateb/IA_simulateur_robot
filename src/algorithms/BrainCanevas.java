/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/BrainCanevas.java 2014-10-19 buixuan.
 * ******************************************************/
package algorithms;

import characteristics.Parameters;
import robotsimulator.Brain;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class BrainCanevas extends Brain {
    private static final double HEADINGPRECISION = 0.001;
    private static final double ANGLEPRECISION = 0.1;

    private static final int ROCKY = 0x1EADDA;
    private static final int MARIO = 0x5EC0;
    private static final int UNDEFINED = 0xBADC0DE0;

    private static final int TURNLEFTTASK = 1;
    private static final int MOVETASK = 2;
    private static final int TURNRIGHTTASK = 3;
    // Pour le stage 2, l'état SINK sera utilisé lorsque le tour est terminé
    private static final int SINK = 0xBADC0DE1;

    private static IFrontSensorResult.Types WALL = IFrontSensorResult.Types.WALL;
    private static IFrontSensorResult.Types TEAMMAIN = IFrontSensorResult.Types.TeamMainBot;

    private boolean isTurning = true; // Pour s'assurer qu'il ne bloque pas en activate()

    private int state;
    private double oldAngle;
    private double myX, myY;
    private boolean isMoving;
    private int whoAmI;

    // Variables pour l'odométrie et la mesure du terrain (stage 2)
    private double startX, startY;
    private double minX, maxX, minY, maxY;
    private boolean hasLeftStart = false; // Pour s'assurer que le robot s'est éloigné du point de départ
    private boolean loopCompleted = false; // Tour complet du terrain

    public BrainCanevas() {
        super();
    }

    public void activate() {
        // ODOMETRY CODE
        whoAmI = ROCKY;
        for (IRadarResult o : detectRadar())
            if (isSameDirection(o.getObjectDirection(), Parameters.NORTH))
                whoAmI = MARIO;
        if (whoAmI == ROCKY) {
            myX = Parameters.teamASecondaryBot1InitX;
            myY = Parameters.teamASecondaryBot1InitY;
        } else {
            myX = 0;
            myY = 0;
        }

        // Initialisation de l'odométrie pour mesurer le terrain
        startX = myX;
        startY = myY;
        minX = myX;
        maxX = myX;
        minY = myY;
        maxY = myY;
        hasLeftStart = false;
        loopCompleted = false;

        // INIT : On démarre en suivant le mur (stage 1)
        state = (whoAmI == ROCKY) ? TURNLEFTTASK : SINK;
        isMoving = false;
        oldAngle = getHeading();
    }

    public void step() {
        // Mise à jour de l'odométrie à chaque mouvement (pour le robot ROCKY)
        if (isMoving && whoAmI == ROCKY) {
            double stepSize = Parameters.teamASecondaryBotSpeed;
            double heading = getHeading();
            myX += stepSize * Math.cos(heading);
            myY += stepSize * Math.sin(heading);
            // Mise à jour des bornes
            minX = Math.min(minX, myX);
            maxX = Math.max(maxX, myX);
            minY = Math.min(minY, myY);
            maxY = Math.max(maxY, myY);
            isMoving = false;
        }

        // Affichage de la position courante
        if (whoAmI == ROCKY) {
            sendLogMessage("#ROCKY se trouve en (" + (int) myX + ", " + (int) myY + ").");
        }

        // Vérification de la complétion du tour pour le stage 2
        if (whoAmI == ROCKY && !loopCompleted) {
            double distFromStart = Math.hypot(myX - startX, myY - startY);
            if (!hasLeftStart && distFromStart > 20) { // seuil arbitraire pour dire "on a quitté le point de départ"
                hasLeftStart = true;
            }
            // Si on est repassé près du point de départ après être parti
            if (hasLeftStart && distFromStart < 10) { // seuil pour considérer qu'on revient au point initial
                loopCompleted = true;
                state = SINK; // On passe en état final
                double width = maxX - minX;
                double height = maxY - minY;
                sendLogMessage("Dimensions du terrain : largeur = " + (int) width + ", hauteur = " + (int) height);
            }
        }

        // AUTOMATON
        if (state == TURNLEFTTASK && !(isSameDirection(getHeading(), Parameters.NORTH))) {
            stepTurn(Parameters.Direction.LEFT);
            //sendLogMessage("Orientation initiale : tournant à gauche pour atteindre le Nord.");
            return;
        }
        if (state == TURNLEFTTASK && isSameDirection(getHeading(), Parameters.NORTH)) {
            state = MOVETASK;
            myMove();
            //sendLogMessage("Orientation atteinte (Nord). Avancer.");
            return;
        }
        if (state == MOVETASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
            myMove(); // avancer tant qu'il n'y a pas de mur
            //sendLogMessage("Avancer.");
            return;
        }
        if (state == MOVETASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
            state = TURNRIGHTTASK;
            oldAngle = getHeading();
            stepTurn(Parameters.Direction.RIGHT);
            //sendLogMessage("Mur détecté. Tournant à droite.");
            return;
        }
        if (state == TURNRIGHTTASK && !(isSameDirection(getHeading(), oldAngle + Parameters.RIGHTTURNFULLANGLE))) {
            stepTurn(Parameters.Direction.RIGHT);
            //sendLogMessage("Tournant à droite.");
            return;
        }
        if (state == TURNRIGHTTASK && isSameDirection(getHeading(), oldAngle + Parameters.RIGHTTURNFULLANGLE)) {
            state = MOVETASK;
            myMove();
            //sendLogMessage("Nouvelle direction validée. Avancer.");
            return;
        }

        // État final SINK : pour le stage 2, on arrête de bouger une fois les dimensions déterminées
        if (state == SINK) {
            if (loopCompleted) {
                // Ne rien faire, on a fini et les dimensions ont été affichées.
                return;
            } else {
                myMove();
                return;
            }
        }

        // Bloc par défaut (optionnel)
        if (true) {
            return;
        }
    }

    private void myMove(){
        isMoving = true;
        move();
    }

    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(normalize(dir1) - normalize(dir2)) < ANGLEPRECISION;
    }

    private double normalize(double dir){
        double res = dir;
        while (res < 0) res += 2 * Math.PI;
        while (res >= 2 * Math.PI) res -= 2 * Math.PI;
        return res;
    }
}
