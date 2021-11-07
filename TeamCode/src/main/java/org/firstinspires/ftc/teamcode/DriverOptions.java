package org.firstinspires.ftc.teamcode;

    public class DriverOptions {
        private int delay;
        private String startingPositionModes;
        private String parking;
        private String deliveryRoutes;
        private String duckPosition;

        public String getStartingPositionModes() {
            return startingPositionModes;
        }

        public void setStartingPositionModes(String startingPositionModes) {
            this.startingPositionModes = startingPositionModes;
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



        public String getDuckPosition() {
            return duckPosition;
        }

        public void setDuckPosition(String isParkOnly) {
            this.duckPosition = isParkOnly;
        }

        public int getDelay() {
            return delay;
        }

        public void setDelay(int delay) {
            this.delay = delay;
        }
    }
