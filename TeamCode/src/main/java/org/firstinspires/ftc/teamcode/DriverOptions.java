package org.firstinspires.ftc.teamcode;

public class DriverOptions {
    private int delay;
    private String startingPositionModes;
    private String parking;
    private int delayParking;
    private int alliance;
    private String parkingOnly;
    private int freightDeliveryCount;
    private boolean isDuckParkingCCW;
    private boolean deliver_to_hub_using_opencv;

    public boolean isDeliver_to_hub_using_opencv() {
        return deliver_to_hub_using_opencv;
    }

    public void setDeliver_to_hub_using_opencv(boolean deliver_to_hub_using_opencv) {
        this.deliver_to_hub_using_opencv = deliver_to_hub_using_opencv;
    }


    public boolean isDuckParkingCCW() {
        return isDuckParkingCCW;
    }

    public void setDuckParkingDirection(boolean duckParkingCCW) {
        this.isDuckParkingCCW = duckParkingCCW;
    }



    public String getParkingOnly() {
        return parkingOnly;
    }

    public void setParkingOnly(String parkingOnly) {
        this.parkingOnly = parkingOnly;
    }

    public int getFreightDeliveryCount() { return freightDeliveryCount; }

    public void setFreightDeliveryCount(int freightDeliveryCount) {
        this.freightDeliveryCount = freightDeliveryCount;
    }

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

}
