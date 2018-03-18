package chartadvancedscatter;

public class Spherical3 {
        private static double CLOSE_ENOUGH_TO_ZERO = 0.000000000000001;
    
	public double r;
	public double az;
	public double el;

	public Spherical3 () {
            this.set(0, 0, 0);
	}

	public Spherical3 (double az, double el, double r) {
		this.set(az, el, r);
	}

	public Spherical3 (final Spherical3 inVector) {
		this.set(inVector);
	}

	public String toString () {
		return "(" + az + "," + el + "," + r + ")";
	}
	
	public Spherical3 set (double az, double el, double r) {
		this.az = az;
		this.el = el;
		this.r = r;
		return this;
	}

	public Spherical3 set (final Spherical3 inVector) {
		return this.set(inVector.az, inVector.el, inVector.r);
	}

	public Spherical3 setFromVector3 (final Vector3 inV) {
            double range = Math.sqrt((inV.x*inV.x)+(inV.y*inV.y)+(inV.z*inV.z));
            double elevation = Math.atan(inV.z/inV.x);
            double azimuth = Math.asin(inV.y/range);
            return this.set(azimuth, elevation, range);
	}

	public Spherical3 copy () {
		return new Spherical3(this);
	}

}
