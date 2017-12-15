/*******************************************************************************
 * Copyright (c) 2007-2010, Geophysical Center RAS
 * Sergey Berezin, Dmitry Voytsekhovskiy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided
 * that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * - Neither the name of the Geophysical Center RAS nor the names
 * of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using System.Windows;
using System.Globalization;

namespace Common
{
    /// <summary>
    /// Represents a pair of spherical coordinates: latitude and longitude, and thus defines a point on a sphere.
    /// </summary>
    public struct GeoPoint
    {
        /// <summary>
        /// Range of latitude/longitude values.
        /// </summary>
        public enum Range : byte
        {
            /// <summary>Latitude: -90 .. 90</summary>
            RangeLat90 = 1,
            /// <summary>Latitude: 0 .. 180</summary>
            RangeLat180 = 2,
            /// <summary>Longitude: -180 .. 180</summary>
            RangeLong180 = 4,
            /// <summary>Longitude: 0 .. 360</summary>
            RangeLong360 = 8
        }

        // Default: RangeLat90 | RangeLong180
        public const double LatitudeMin = -90.0f;
        public const double LatitudeMax = 90.0f;
        public const double LongitudeMin = -180.0f;
        public const double LongitudeMax = 180.0f;

        private const double DegToRad = System.Math.PI / 180.0;
        private const double RadToDeg = 180.0 / System.Math.PI;

        /// <summary>
        /// Gets an empty point.
        /// </summary>
        public static GeoPoint Empty
        {
            get { return new GeoPoint(0.0f, 0.0f); }
        }

        public static double GetWidthLongitude(GeoPoint p1, GeoPoint p2)
        {
            double w = p2.Longitude - p1.Longitude;
            return (w <= 0) ? (360 + w) : w;
        }

        public static double GetHeightLatitude(GeoPoint p1, GeoPoint p2)
        {
            return Math.Abs(p2.Latitude - p1.Latitude);
        }

        private double lat; // in degrees
        private double lon; // in degrees

        /// <summary>
        /// Initializes a new GeoPoint.
        /// </summary>
        /// <param name="latitude">Latitude of the point in degrees.</param>
        /// <param name="longitude">Longitude of the point in degrees.</param>
        public GeoPoint(double latitude, double longitude)
        {
            this.lat = this.lon = 0.0f;

            Latitude = latitude;
            Longitude = longitude;
        }

        /// <summary>
        /// Initializes a new GeoPoint.
        /// </summary>
        /// <param name="latitude">Latitude of the point.</param>
        /// <param name="longitude">Longitude of the point.</param>
        /// <param name="inRadians">If true then values are in radians, otherwise in degrees.</param>
        public GeoPoint(double latitude, double longitude, bool inRadians)
        {
            this.lat = this.lon = 0.0f;

            if (inRadians)
            {
                Latitude = (double)(latitude * RadToDeg);
                Longitude = (double)(longitude * RadToDeg);
            }
            else
            {
                Latitude = latitude;
                Longitude = longitude;
            }
        }

        /// <summary>
        /// Initializes a new GeoPoint.
        /// </summary>
        /// <param name="latitude">Latitude of the point in degrees.</param>
        /// <param name="longitude">Longitude of the point in degrees.</param>
        /// <param name="inputRange">Range type of input values.</param>
        public GeoPoint(double latitude, double longitude, Range inputRange)
        {
            this.lat = this.lon = 0.0f;

            if ((inputRange & Range.RangeLat180) != 0)
            {
                latitude -= 90;
            }
            if ((inputRange & Range.RangeLong360) != 0)
            {
                longitude -= 360;
            }

            Latitude = latitude;
            Longitude = longitude;
        }

        /// <summary>
        /// Initializes a new GeoPoint.
        /// </summary>
        /// <param name="latitude">Latitude of the point.</param>
        /// <param name="longitude">Longitude of the point.</param>
        /// <param name="inRadians">If true then values are in radians, otherwise in degrees.</param>
        /// <param name="inputRange">Range type of input values.</param>
        public GeoPoint(double latitude, double longitude, bool inRadians, Range inputRange)
        {
            this.lat = this.lon = 0.0f;

            if ((inputRange & Range.RangeLat180) != 0)
            {
                latitude -= inRadians ? (double)(Math.PI / 2) : 90;
            }
            if ((inputRange & Range.RangeLong360) != 0)
            {
                longitude -= inRadians ? (double)(2 * Math.PI) : 360;
            }


            if (inRadians)
            {
                Latitude = (double)(latitude * RadToDeg);
                Longitude = (double)(longitude * RadToDeg);
            }
            else
            {
                Latitude = latitude;
                Longitude = longitude;
            }
        }

        /// <summary>
        /// Gets or sets the latitude value in degrees.
        /// </summary>
        public double Latitude
        {
            get { return lat; }
            set
            {
                if (value >= LatitudeMax)
                    lat = LatitudeMax;
                else if (value < LatitudeMin)
                    lat = LatitudeMin;
                else
                    lat = value;
            }
        }

        /// <summary>
        /// Gets or sets the longitude value in degrees.
        /// </summary>
        public double Longitude
        {
            get { return lon; }
            set
            {
                if (value >= LongitudeMax)
                    lon = (value - LongitudeMin) % (LongitudeMax - LongitudeMin) + LongitudeMin;
                else if (value < LongitudeMin)
                    lon = LongitudeMax - (LongitudeMin - value) % (LongitudeMax - LongitudeMin);
                else
                    lon = value;
            }
        }

        public override string ToString()
        {
            return string.Format("{0},{1}", lat.ToString(CultureInfo.InvariantCulture), lon.ToString(CultureInfo.InvariantCulture));
        }

        public string ToStringEx()
        {
            return string.Format("Lat: {0}В° Long: {1}В°", lat, lon);
        }

        public string ToStringEx2()
        {
            return String.Format(CultureInfo.InvariantCulture, "{0:F}{1} {2:F}{3}", 
                Math.Abs(lat), lat >= 0 ? 'N' : 'S', 
                Math.Abs(lon), lon >= 0 ? 'E' : 'W');
        }

        public static GeoPoint ParseEx2(string s)
        {
            if (s == null) throw new ArgumentNullException("s");

            var items = s.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (items.Length != 2) throw new NotSupportedException("String is incorrect");

            string slat = items[0].Trim();
            char ch = Char.ToUpper(slat[slat.Length - 1]);
            double lat = double.Parse(slat.Substring(0, slat.Length - 1), CultureInfo.InvariantCulture);
            if (ch == 'S') lat = -lat;
            else if (ch != 'N') throw new NotSupportedException("String is incorrect");

            string slon = items[1].Trim();
            ch = Char.ToUpper(slon[slon.Length - 1]);
            double lon = double.Parse(slon.Substring(0, slon.Length - 1), CultureInfo.InvariantCulture);
            if (ch == 'W') lon = -lon;
            else if (ch != 'E') throw new NotSupportedException("String is incorrect");
            
            return new GeoPoint(lat, lon, false, GeoPoint.Range.RangeLong180 | GeoPoint.Range.RangeLat90);
        }

        public static Vector3D ToCartesian(GeoPoint point, double radius)
        {
            double latitude = point.Latitude * DegToRad;
            double longitude = point.Longitude * DegToRad;

            double radCosLat = radius * Math.Cos(latitude);

            return new Vector3D(
                (radCosLat * Math.Cos(longitude)),
                (radCosLat * Math.Sin(longitude)),
                (radius * Math.Sin(latitude)));
        }

        public static GeoPoint FromCartesian(double x, double y, double z)
        {
            double rho = Math.Sqrt((double)(x * x + y * y + z * z));
            if (rho == 0)
                return GeoPoint.Empty;

            double longitude = (double)Math.Atan2(y, x);
            double latitude = (double)(Math.Asin(z / rho));

            return new GeoPoint(latitude, longitude, true);
        }

        public static GeoPoint FromCartesian(Vector3D v)
        {
            return GeoPoint.FromCartesian((double)v.X, (double)v.Y, (double)v.Z);
        }

        public static void CircleDistance(GeoPoint p1, GeoPoint p2, out double latDistance, out double longDistance)
        {
            double lon = Math.Abs(p2.Longitude - p1.Longitude);

            if (lon > 180)
                lon = 360.0f - lon;

            latDistance =  Math.Abs(p2.Latitude - p1.Latitude);
            longDistance = lon;
        }

        public static double DistanceOverSurface(GeoPoint p1, GeoPoint p2)
        {
            double lat1 = p1.lat * DegToRad;
            double lat2 = p2.lat * DegToRad;
            double lon1 = p1.lon * DegToRad;
            double lon2 = p2.lon * DegToRad;            
            return Math.Acos(Math.Sin(lat1) * Math.Sin(lat2) + Math.Cos(lat1) * Math.Cos(lat2) * Math.Cos(lon2 - lon2));           
        }

        public static bool operator ==(GeoPoint p1, GeoPoint p2)
        {
            return p1.Latitude == p2.Latitude && p1.Longitude == p2.Longitude;
        }

        public static bool operator !=(GeoPoint p1, GeoPoint p2)
        {
            return p1.Latitude != p2.Latitude || p1.Longitude != p2.Longitude;
        }

        public override bool Equals(object obj)
        {
            if (obj is GeoPoint)
            {
                return this == (GeoPoint)obj;
            }

            return false;
        }

        public override int GetHashCode()
        {
            return Longitude.GetHashCode() ^ Latitude.GetHashCode();
        }

        public GeoPoint Max(GeoPoint p2)
        {
            return new GeoPoint(Math.Max(lat, p2.lat), Math.Max(lon, p2.lon));
        }

        public GeoPoint Min(GeoPoint p2)
        {
            return new GeoPoint(Math.Min(lat, p2.lat), Math.Min(lon, p2.lon));
        }
    }

    public struct GeoRect
    {
        private GeoPoint leftBottom;
        private GeoPoint rightTop;
        private bool empty;

        public GeoRect(double leftLong, double rightLong, double bottomLat, double topLat)
        {
            leftBottom = new GeoPoint(bottomLat, leftLong);
            rightTop = new GeoPoint(topLat, rightLong);
            empty = false;
        }

        public GeoRect(GeoPoint lowerLeft, GeoPoint upperRight)
        {
            leftBottom = lowerLeft;
            rightTop = upperRight;
            empty = false;
        }

        private GeoRect(bool empty)
        {
            leftBottom = rightTop = GeoPoint.Empty;
            this.empty = empty;
        }

        public IEnumerable<GeoPoint> Points
        {
            get
            {
                yield return leftBottom;
                yield return new GeoPoint(leftBottom.Latitude, rightTop.Longitude);
                yield return rightTop;
                yield return new GeoPoint(rightTop.Latitude, leftBottom.Longitude);
            }
        }

        public bool IsEmpty
        {
            get { return empty; }
        }

        public static GeoRect Empty
        {
            get { return new GeoRect(true); }
        }

        public GeoPoint LowerLeft
        {
            get { return leftBottom; }
        }

        public GeoPoint UpperRight
        {
            get { return rightTop; }
        }

        public double LeftLongitude
        {
            get
            {
                return leftBottom.Longitude;
            }
        }

        public double RightLongitude
        {
            get
            {
                if (rightTop.Longitude == -180)
                    return 180;

                return rightTop.Longitude;
            }
        }

        public double BottomLatitude
        {
            get
            {
                return leftBottom.Latitude;
            }
        }

        public double TopLatitude
        {
            get
            {
                return rightTop.Latitude;
            }
        }

        public double WidthLongitude
        {
            get
            {
                double w = rightTop.Longitude - leftBottom.Longitude;
                return (w <= 0) ? (360 + w) : w;
            }
        }

        public double HeightLatitude
        {
            get
            {
                return Math.Abs(rightTop.Latitude - leftBottom.Latitude);
            }
        }

        /// <summary>
        /// Inflates GeoRect by the specified percentage.
        /// </summary>
        /// <param name="rect">The GeoRect to inflate.</param>
        /// <param name="dpx">[New width] = [old width] * dpx</param>
        /// <param name="dpy">[New height] = [old height] * dpx</param>
        /// <returns>Inflated GeoRect.</returns>
        public static GeoRect Inflate(GeoRect rect, double dpx, double dpy)
        {
            if (rect.IsEmpty)
                return rect;

            double longIncr = 0.5f *
                Math.Min((double)(rect.WidthLongitude * (dpx - 1.0)),
                    359.99999f - rect.WidthLongitude);
            double latIncr = (double)(rect.HeightLatitude * (dpy - 1.0) * 0.5);

            return new GeoRect(rect.LeftLongitude - longIncr,
                rect.RightLongitude + longIncr,
                rect.BottomLatitude - latIncr,
                rect.TopLatitude + latIncr);
        }


        public bool GoesThrough180
        {
            get
            {
                return LeftLongitude > RightLongitude;
            }
        }

        public override string ToString()
        {
            if (IsEmpty)
                return "<empty>";
            return string.Format("(Lower-left: {0} Upper-right: {1})",
                leftBottom, rightTop);
        }

        public static GeoRect[] Intersect(GeoRect rect1, GeoRect rect2)
        {
            if (rect1.IsEmpty || rect2.IsEmpty)
                return new GeoRect[0];

            GeoRect[] rects1 = rect1.GetSimpleRects();
            GeoRect[] rects2 = rect2.GetSimpleRects();
            GeoRect r1 = GeoRect.Empty;
            GeoRect r2 = GeoRect.Empty; // TODO: here can be more than 2 segments!

            for (int i = 0; i < rects1.Length; i++)
            {
                Rect ri = new Rect(rects1[i].LeftLongitude, rects1[i].BottomLatitude,
                    rects1[i].WidthLongitude, rects1[i].HeightLatitude);

                for (int j = 0; j < rects2.Length; j++)
                {
                    Rect rj = new Rect(rects2[j].LeftLongitude, rects2[j].BottomLatitude,
                    rects2[j].WidthLongitude, rects2[j].HeightLatitude);

                    rj = Rect.Intersect(ri, rj);
                    if (!rj.IsEmpty)
                    {
                        GeoRect r = new GeoRect(rj.Left, rj.Right, rj.Y, rj.Bottom);
                        if (r1.IsEmpty)
                            r1 = r;
                        else
                            r2 = r;
                    }
                }
            }

            if (r1.IsEmpty)
            {
                if (r2.IsEmpty)
                    return new GeoRect[0];
                return new GeoRect[1] { r2 };
            }
            else
            {
                if (r2.IsEmpty)
                    return new GeoRect[1] { r1 };

                if (r1.RightLongitude == -180 || r2.LeftLongitude == -180)
                {
                    return new GeoRect[1] { 
                        new GeoRect(r1.LeftLongitude, r2.RightLongitude, r1.BottomLatitude, r1.TopLatitude)
                    };
                }
                else if (r2.RightLongitude == -180 || r1.LeftLongitude == -180)
                {
                    return new GeoRect[1] { 
                        new GeoRect(r2.LeftLongitude, r1.RightLongitude, r1.BottomLatitude, r1.TopLatitude)
                    };
                }
                else
                    return new GeoRect[2] { r1, r2 };
            }
        }

        /// <summary>
        /// If the GeoRect goes through the 180 degrees by longitude,
        /// then returns this rect as two segments: 1st before 180, 2nd - behind it.
        /// Otherwise returns the copy of the entire rect.
        /// </summary>
        public GeoRect[] GetSimpleRects()
        {
            if (!GoesThrough180)
                return new GeoRect[] { this };

            return new GeoRect[] {
                new GeoRect(leftBottom.Longitude, 180, leftBottom.Latitude, rightTop.Latitude),
                new GeoRect(-180, rightTop.Longitude, leftBottom.Latitude, rightTop.Latitude)};
        }

        public bool Contains(GeoPoint p)
        {
            if (this.IsEmpty) return false;

            if (p.Latitude > this.TopLatitude || p.Latitude < this.BottomLatitude) return false;

            double left = LeftLongitude;
            double right = RightLongitude;
            if (left <= right)
            {
                if (left == -180 && p.Longitude == -180)
                    return false;

                return (p.Longitude >= left) && (p.Longitude <= right);
            }
            else
            {
                return (left <= p.Longitude) || (right >= p.Longitude);
            }
        }

        public bool Contains(GeoRect rect)
        {
            if (this.IsEmpty)
                return false;
            if (rect.IsEmpty)
                return true;

            if (!Contains(BottomLatitude, TopLatitude, rect.BottomLatitude, rect.TopLatitude))
                return false;

            double left = LeftLongitude;
            double right = RightLongitude;
            if (left <= right)
            {
                if (rect.GoesThrough180)
                    return false;

                return Contains(left, right, rect.LeftLongitude, rect.RightLongitude);
            }
            else
            {
                if (rect.GoesThrough180)
                    return left <= rect.LeftLongitude &&
                        right >= rect.RightLongitude;
                return (left <= rect.LeftLongitude && left <= rect.RightLongitude) ||
                    (right >= rect.RightLongitude && right >= rect.LeftLongitude);
            }
        }

        private bool Contains(double outer1, double outer2, double inner1, double inner2)
        {
            return outer1 <= inner1 && outer2 >= inner2;
        }

        public static bool operator ==(GeoRect r1, GeoRect r2)
        {
            return (r1.IsEmpty && r2.IsEmpty) ||
                (r1.leftBottom == r2.leftBottom && r1.rightTop == r2.rightTop);
        }

        public static bool operator !=(GeoRect r1, GeoRect r2)
        {
            if (r1.IsEmpty && r2.IsEmpty)
                return false;

            return
                r1.leftBottom != r2.leftBottom || r1.rightTop != r2.rightTop;
        }

        public override int GetHashCode()
        {
            return leftBottom.GetHashCode() ^ rightTop.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            if (obj is GeoRect)
            {
                return this == (GeoRect)obj;
            }

            return false;
        }
    }

    /// <summary>
    /// GeoQuad is a convex quad at lat/lon coordinates.
    /// </summary>
    public class GeoQuad
    {
        private GeoPoint p1;
        private GeoPoint p2;
        private GeoPoint p3;
        private GeoPoint p4;

        public GeoQuad(double lat0, double lon0, double lat1, double lon1, double lat2, double lon2,
            double lat3, double lon3)
        {
            p1 = new GeoPoint(lat0, lon0);
            p2 = new GeoPoint(lat1, lon1);
            p3 = new GeoPoint(lat2, lon2);
            p4 = new GeoPoint(lat3, lon3);
        }

        public GeoQuad(params GeoPoint[] points)
        {
            if (points.Length != 4)
                throw new Exception("Incorrect number of points");

            p1 = points[0];
            p2 = points[1];
            p3 = points[2];
            p4 = points[3];
        }

        public GeoPoint this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return p1;
                    case 1: return p2;
                    case 2: return p3;
                    case 3: return p4;
                    default: throw new ApplicationException("Point index is out of range");
                }
            }
        }

        public IEnumerable<GeoPoint[]> Segments
        {
            get
            {
                GeoPoint[] segment = new GeoPoint[2];
                segment[0] = p1; segment[1] = p2;
                yield return segment;
                segment[0] = p2; segment[1] = p3;
                yield return segment;
                segment[0] = p3; segment[1] = p4;
                yield return segment;
                segment[0] = p4; segment[1] = p1;
                yield return segment;
            }
        }

        /// <summary>
        /// Tries to determine whether the quad passes through the longitude = 180
        /// and if it does then divides it in two polygons.
        /// </summary>
        /// <param name="part1"></param>
        /// <param name="part2"></param>
        public void GetSafeDivisions(out GeoConvexPolygon polygon1, out GeoConvexPolygon polygon2)
        {
            bool exchange = false;

            List<GeoPoint> part1 = new List<GeoPoint>(5), part2 = new List<GeoPoint>(5);
            part1.Add(this[0]);
            GeoPoint p1;
            GeoPoint p2 = GeoPoint.Empty;

            for (int i = 1; i < 4; i++)
            {
                p1 = this[i - 1];
                p2 = this[i];


                if ((p2.Longitude > 0 && p1.Longitude < 0) &&
                    (exchange || (p2.Longitude - p1.Longitude > 360 + p1.Longitude - p2.Longitude)))
                {
                    GeoPoint p = new GeoPoint(
                        (double)IntersectionWithBound(360 + p1.Longitude, p1.Latitude, p2.Longitude, p2.Latitude, 180),
                        -180);
                    part1.Add(p);
                    List<GeoPoint> temp = part1; part1 = part2; part2 = temp;
                    part1.Add(p);
                    exchange = !exchange;
                }
                else if ((p2.Longitude < 0 && p1.Longitude > 0) &&
                    (exchange || (p1.Longitude - p2.Longitude > 360 + p2.Longitude - p1.Longitude)))
                {
                    GeoPoint p = new GeoPoint(
                        (double)IntersectionWithBound(p1.Longitude, p1.Latitude, 360 + p2.Longitude, p2.Latitude, 180),
                        180);
                    part1.Add(p);
                    List<GeoPoint> temp = part1; part1 = part2; part2 = temp;
                    part1.Add(p);
                    exchange = !exchange;
                }


                part1.Add(p2);
            }

            if (exchange)
            {
                p1 = p2; p2 = this[0];
                if ((p2.Longitude >= 0 && p1.Longitude < 0) &&
                    (exchange || (p2.Longitude - p1.Longitude > 360 + p1.Longitude - p2.Longitude)))
                {
                    GeoPoint p = new GeoPoint(
                        (double)IntersectionWithBound(360 + p1.Longitude, p1.Latitude, p2.Longitude, p2.Latitude, 180),
                        -180);
                    part1.Add(p);
                    List<GeoPoint> temp = part1; part1 = part2; part2 = temp;
                    part1.Add(p);
                    exchange = !exchange;
                }
                else if ((p2.Longitude < 0 && p1.Longitude >= 0) &&
                    (exchange || (p1.Longitude - p2.Longitude > 360 + p2.Longitude - p1.Longitude)))
                {
                    GeoPoint p = new GeoPoint(
                        (double)IntersectionWithBound(p1.Longitude, p1.Latitude, 360 + p2.Longitude, p2.Latitude, 180),
                        180);
                    part1.Add(p);
                    List<GeoPoint> temp = part1; part1 = part2; part2 = temp;
                    part1.Add(p);
                    exchange = !exchange;
                }
                else
                {
                    System.Diagnostics.Trace.WriteLine("Unexpected case in the algorithm");
                    part1.Clear();
                    part2.Clear();
                }
            }

            polygon1 = new GeoConvexPolygon(part1);
            polygon2 = new GeoConvexPolygon(part2);
        }

        private double IntersectionWithBound(double x1, double y1, double x2, double y2, double boundX)
        {
            return (y2 - y1) * (boundX - x1) / (x2 - x1) + y1;
        }


        public bool Intersects(GeoRect rect)
        {
            if (rect.GoesThrough180)
            {
                GeoRect[] rects = rect.GetSimpleRects();
                return this.Intersects(rects[0]) || this.Intersects(rects[1]);
            }

            // 1. Simple filtering
            double Ymin1 = Min(p1.Latitude, p2.Latitude, p3.Latitude, p4.Latitude);
            double Ymax1 = Max(p1.Latitude, p2.Latitude, p3.Latitude, p4.Latitude);
            double Ymin2 = Math.Min(rect.BottomLatitude, rect.TopLatitude);
            double Ymax2 = Math.Max(rect.BottomLatitude, rect.TopLatitude);

            if (!IntersectsSegments(Ymin1, Ymax1, Ymin2, Ymax2))
                return false;

            double Xmin1 = Min(p1.Longitude, p2.Longitude, p3.Longitude, p4.Longitude);
            double Xmax1 = Max(p1.Longitude, p2.Longitude, p3.Longitude, p4.Longitude);
            double Xmin2 = Math.Min(rect.LeftLongitude, rect.RightLongitude);
            double Xmax2 = Math.Max(rect.LeftLongitude, rect.RightLongitude);

            if (!IntersectsSegments(Xmin1, Xmax1, Xmin2, Xmax2))
                return false;

            // 2. Quad is within rect
            if (Xmin1 >= Xmin2 && Xmax1 <= Xmax2 && Ymin1 >= Ymin2 && Ymax1 <= Ymax2)
                return true;

            // 3. Partial Intersection
            // Any or all points of rect is inside quad
            foreach (GeoPoint p in rect.Points)
            {
                bool b = Contains(p.Latitude, p.Longitude);
                if (b) return true;
            }

            // All Points are outside
            foreach (GeoPoint[] segment in this.Segments)
            {
                if (IntersectsVertical(segment[0], segment[1], Xmin2, Ymin2, Ymax2))
                    return true;
                if (IntersectsVertical(segment[0], segment[1], Xmax2, Ymin2, Ymax2))
                    return true;
                if (IntersectsHorizontal(segment[0], segment[1], Xmin2, Xmax2, Ymin2))
                    return true;
                if (IntersectsHorizontal(segment[0], segment[1], Xmin2, Xmax2, Ymax2))
                    return true;
            }
            return false;
        }

        private bool ToTheLeftOf(GeoPoint a, GeoPoint b, double x, double y)
        {
            if (a.Latitude == b.Latitude)
            {
                if (a.Longitude == b.Longitude) return false;
                if (a.Longitude >= b.Longitude)
                    return y >= a.Latitude;
                else
                    return y <= a.Latitude;
            }
            if (a.Longitude == b.Longitude)
            {
                if (a.Latitude > b.Latitude)
                    return x <= a.Longitude;
                else
                    return x >= a.Longitude;
            }
            return ((x - a.Longitude) * (b.Latitude - a.Latitude) -
                (y - a.Latitude) * (b.Longitude - a.Longitude)) >= 0;
        }

        private static bool IntersectsSegments(double a, double b, double s, double t)
        {
            if (a > b)
            {
                double x = a; a = b; b = x;
            }
            if (s > t)
            {
                double x = s; s = t; t = x;
            }
            return ((a <= t && b >= s) || (s <= a && t >= b));
        }

        private static bool IntersectsVertical(GeoPoint c, GeoPoint d, double ax, double ay, double by)
        {
            if (d.Longitude == c.Longitude)
            {
                if (ax != d.Longitude) return false;
                return IntersectsSegments(c.Latitude, d.Latitude, ay, by);
            }

            double r = ((ay - c.Latitude) * (d.Longitude - c.Longitude) - (ax - c.Longitude) * (d.Latitude - c.Latitude))
               / (-(by - ay) * (d.Longitude - c.Longitude));
            if (r < 0 || r > 1) return false;

            double s = ((ax - c.Longitude) / (d.Longitude - c.Longitude));
            return (s >= 0) && (s <= 1);
        }

        private static bool IntersectsHorizontal(GeoPoint c, GeoPoint d, double ax, double bx, double ay)
        {
            if (d.Latitude == c.Latitude)
            {
                if (ay != c.Latitude) return false;
                return IntersectsSegments(c.Longitude, d.Longitude, ax, bx);
            }

            double r = ((ay - c.Latitude) * (d.Longitude - c.Longitude) - (ax - c.Longitude) * (d.Latitude - c.Latitude))
                / ((bx - ax) * (d.Latitude - c.Latitude));
            if (r < 0 || r > 1) return false;

            double s = ((ay - c.Latitude) / (d.Latitude - c.Latitude));
            return (s >= 0) && (s <= 1);
        }

        private static double Min(double a, double b, double c, double d)
        {
            return Math.Min(Math.Min(a, b), Math.Min(c, d));
        }

        private static double Max(double a, double b, double c, double d)
        {
            return Math.Max(Math.Max(a, b), Math.Max(c, d));
        }

        public bool Contains(double lat, double lon)
        {
            bool left = true;
            bool right = true;
            foreach (GeoPoint[] segment in this.Segments)
            {
                bool b = ToTheLeftOf(segment[0], segment[1], lon, lat);
                left = left && b;
                right = right && !b;

                if (!(left || right)) break;
            }
            return left || right;
        }

        public bool Contains(GeoPoint p)
        {
            return Contains(p.Latitude, p.Longitude);
        }
    }
}
