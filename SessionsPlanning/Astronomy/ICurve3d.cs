using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;
using System.Collections.ObjectModel;

namespace Astronomy
{
    public interface ICurve3d<TTime,TSpan>
    {
        TTime Min { get; }
        TTime Max { get; }

        ICurveCursor3d<TTime, TSpan> GetCursor(TTime time);  
    }

    public interface ICurveCursor3d<TTime,TSpan>
    {
        TTime Time { get; }
        Point3D Position { get; }

        /// <summary>
        /// Moves the time on given time delta (either forward or backward).
        /// </summary>
        /// <param name="timeDelta"></param>
        /// <returns>False, if new time is equal to min/max or is out of the range.</returns>
        /// <remarks>
        /// If <see cref="Move"/> goes outside of the range, it returns false and <see cref="Time"/> corresponds to the closest point,
        /// i.e. first or last.
        /// </remarks>
        bool Move(TSpan timeDelta);
        bool MoveTo(TTime time);
    }

    public interface IOrbitalCurve3d<TTime, TSpan> : ICurve3d<TTime, TSpan>
    {
        IOrbitalCurveCursor3d<TTime, TSpan> SharedCursor { get; }
        IOrbitalCurveCursor3d<TTime, TSpan> GetCursor(TTime time);
       
    }

    public interface IOrbitalCurveCursor3d<TTime, TSpan> : ICurveCursor3d<TTime, TSpan>
    {
        /// <summary>
        /// Gets the vector in GSK which (i) lies in the orbital plane, (ii) is orthogonal to the radius vector of the position.
        /// </summary>
        Vector3D Transversal { get; }
        Quaternion Orientation { get; }
        Vector3D Velocity { get; }
    }
}
