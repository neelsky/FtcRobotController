package org.firstinspires.ftc.teamcode.Neel.subsystems;

public enum Alliance {
    BLUE(38),
    RED(30);

    private final int firstTag;

    Alliance(int firstTag) {
        this.firstTag = firstTag;
    }

    public boolean isHiveTag(int id) {
        return id >= firstTag && id <= firstTag + 7;
    }

    public int toBlueTag(int id) {
        return isHiveTag(id) ? id - firstTag + 38 : -1;
    }
}
