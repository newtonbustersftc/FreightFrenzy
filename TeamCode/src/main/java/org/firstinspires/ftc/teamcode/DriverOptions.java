package org.firstinspires.ftc.teamcode;

    public class DriverOptions {
        private int delay;
        private String startingPositionModes;
        private String parking;
        private int delayParking;
        private String deliveryRoutes;
        private int alliance;

        public void setFreightDeliveryCount(int freightDeliveryCount) {
            this.freightDeliveryCount = freightDeliveryCount;
        }

        private int freightDeliveryCount;

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

        public int getAlliance(){
           return alliance;
        }

        public int getDelayParking() {
            return delayParking;
        }

        public void setDelayParking(int delayParking) {
            this.delayParking = delayParking;
        }

        public int getFreightDeliveryCount() { return freightDeliveryCount; }

    }
