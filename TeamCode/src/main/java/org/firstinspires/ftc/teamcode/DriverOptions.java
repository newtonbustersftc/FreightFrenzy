package org.firstinspires.ftc.teamcode;

    public class DriverOptions {
        private int delay;
        private String startingPositionModes;
        private String parking;
        private String deliveryRoutes;
        private boolean doCarousel;
        private int alliance;

        public String getStartingPositionModes() {
            return startingPositionModes;
        }

        public void setStartingPositionModes(String startingPositionModes) {
            this.startingPositionModes = startingPositionModes;
            if(getStartingPositionModes().contains("RED"))
                this.alliance = 1;
            else
                this.alliance = -1;
        }

        public String getParking() {
            return parking;
        }

        public void setParking(String parking) {
            this.parking = parking;
        }

        public String getDeliveryRoutes() {
            return deliveryRoutes;
        }

        public void setDeliveryRoutes(String deliveryRoutes) {
            this.deliveryRoutes = deliveryRoutes;
        }

        public int getDelay() {
            return delay;
        }

        public void setDelay(int delay) {
            this.delay = delay;
        }

        public void setDoCarousel(boolean doCarousel){ this.doCarousel = doCarousel; }

        public boolean isDoCarousel(){ return doCarousel;}

        public int getAlliance(){
           return alliance;
        }
    }
