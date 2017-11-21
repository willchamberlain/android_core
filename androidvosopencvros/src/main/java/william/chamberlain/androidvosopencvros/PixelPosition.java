package william.chamberlain.androidvosopencvros;

/**
 *  u is in pixels right from the top left of the image, v is in pixels down from the top left of the image.
 */
class PixelPosition {
    private double u;
    private double v;
    private double width;
    private double height;
    public PixelPosition(double u_, double v_, double width_, double height_) {
        this.u=u_; this.v=v_; this.width=width_; this.height=height_;
    }
    public double getU() {
        return u;
    }
    public double getV() {
        return v;
    }
    public double getWidth() {
        return width;
    }
    public double getHeight() {
        return height;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        PixelPosition that = (PixelPosition) o;

        if (Double.compare(that.u, u) != 0) return false;
        if (Double.compare(that.v, v) != 0) return false;
        if (Double.compare(that.width, width) != 0) return false;
        return Double.compare(that.height, height) == 0;

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(u);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(v);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(width);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(height);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "PixelPosition{" +
                "u=" + u +
                ", v=" + v +
                ", width=" + width +
                ", height=" + height +
                '}';
    }

}
