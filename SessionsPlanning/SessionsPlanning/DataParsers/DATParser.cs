using System;
using System.IO;
using System.Text.RegularExpressions;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

using Astronomy;
using SatelliteTrajectory;

namespace DataParsers
{

/// <summary>
/// Parser for DAT file
/// </summary>
public class DatParser
{
    /// <summary>
    /// Load DAT file content into Trajectory object
    /// </summary>
    public static Astronomy.Trajectory getTrajectoryFromDatFile(string datFilename, DateTime dateBegin, DateTime dateEnd)
    { 
        if (!File.Exists(datFilename))
            throw new System.IO.FileNotFoundException("The file" + datFilename + "does not exist", "original");

        TrajectoryPoint[] points;
        DateTime startDateTime = new DateTime();
        double duration = 0;
        double step = 0;
        int pointsNumber = 0;

        Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];

        using (StreamReader sr = File.OpenText(datFilename))
        {
            while (!sr.EndOfStream) // get start DateTime
            {
                string line = sr.ReadLine();
                Regex dateRegx = new Regex("D[ \t]*?=.*T=.*"); // datetime line format
                Match match = dateRegx.Match(line);
                if (match.Success)
                {
                    string strDate = substrVal(line, "D[ \t]*?=", "T");
                    string strTime = substrVal(line, "T[ \t]*?=", "#");

                    startDateTime = DateTime.ParseExact(strDate + strTime, "ddMMyyyy.0HHmmss.ffffff",
                                   System.Globalization.CultureInfo.InvariantCulture);
                    break;
                }
            }

            while (!sr.EndOfStream) // get duration of trajectory 
            {
                string line = sr.ReadLine();
                Regex dateRegx = new Regex("DT[ \t]*?=.*IK.*"); // duration line format
                Match match = dateRegx.Match(line);
                if (match.Success)
                {
                    string strDuration = substrVal(line, "DT[ \t]*?=", "IK");
                    duration = Convert.ToDouble(strDuration.Replace('.', separator));
                    break;
                }
            }

            while (!sr.EndOfStream) // get number of points
            {
                string line = sr.ReadLine();
                Regex dateRegx = new Regex("NN[ \t]*?=[ \t\\d]*(#|$)");
                Match match = dateRegx.Match(line);
                if (match.Success)
                {
                    string strNumber = substrVal(line, "NN[ \t]*?=", "#");
                    pointsNumber = Convert.ToInt16(strNumber);
                    step = duration / pointsNumber;                    
                    break;
                }
            }

            if (step <= 0 || duration <= 0 || pointsNumber <= 0 || startDateTime == null)
                throw new System.ArgumentException("The trajectory data in file" + datFilename + "is incorrect!", "original");

            points = new TrajectoryPoint[pointsNumber];
            
            Point3D position = new Point3D();
            Vector3D velocity = new Vector3D();
            int pointInd = 0;
            while (!sr.EndOfStream)
            {
                string line = sr.ReadLine();
                
                Regex velRegx = new Regex("VX[ \t]*=.*VY[ \t]*=.*VZ[ \t]*=.*"); // velocity line format
                Match match = velRegx.Match(line);

                if (match.Success)
                {
                    string strVX = substrVal(line, "VX[ \t]*?=", "VY");
                    double VX = Convert.ToDouble(strVX.Replace('.', separator));
                    string strVY = substrVal(line, "VY[ \t]*?=", "VZ");
                    double VY = Convert.ToDouble(strVY.Replace('.', separator));
                    string strVZ = substrVal(line, "VZ[ \t]*?=", "#");
                    double VZ = Convert.ToDouble(strVZ.Replace('.', separator));

                    velocity.X = VX;
                    velocity.Y = VY;
                    velocity.Z = VZ;

                    DateTime pointDt = startDateTime.AddSeconds(pointInd * step);

                    if (dateBegin <= pointDt && pointDt <= dateEnd)
                    {
                        TrajectoryPoint point = new TrajectoryPoint(pointDt, position, velocity);
                        points[pointInd] = point;                        
                    }
                    else if (dateEnd < pointDt)
                        break;

                    pointInd++;

                    continue;
                }
                
                Regex posRegx = new Regex("X[ \t]*=.*Y[ \t]*=.*Z[ \t]*=.*"); // position line format
                match = posRegx.Match(line);

                if (match.Success)
                {
                    string strX = substrVal(line, "X[ \t]*?=", "Y");
                    double X = Convert.ToDouble(strX.Replace('.', separator));
                    string strY = substrVal(line, "Y[ \t]*?=", "Z");
                    double Y = Convert.ToDouble(strY.Replace('.', separator));
                    string strZ = substrVal(line, "Z[ \t]*?=", "#");
                    double Z = Convert.ToDouble(strZ.Replace('.', separator));

                    position.X = X;
                    position.Y = Y;
                    position.Z = Z;
                }                               
            }
        }

        return Astronomy.Trajectory.Create(points);
    }

    private static string substrVal(string baseLine, string startLabel, string endLabel)
    {
        Regex regx = new Regex(startLabel + ".*?(" + endLabel + "|$)");
        Match compMatch = regx.Match(baseLine);
        string valStr = compMatch.ToString();
        int startSubstr = valStr.IndexOf(startLabel[startLabel.Length - 1]) + 1;
        int lengthSubstr = valStr.LastIndexOf(endLabel[0]) - startSubstr;
        if (lengthSubstr < 0)
            valStr = valStr.Substring(startSubstr).Trim();
        else
            valStr = valStr.Substring(startSubstr, lengthSubstr).Trim();
        return valStr;
    }
}


}