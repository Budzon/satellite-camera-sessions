#define SPHERE

using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using SatelliteSessions;

namespace GUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public TransformMatrix m_transformMatrix = new TransformMatrix();
        public int m_nChartModelIndex = -1;
#if SPHERE
        public Ellipse3D Terra;
#else
        public EllipseRegion3D Terra;
#endif
        private ViewModel.EarthSatelliteViewModel vm;

        public MainWindow()
        {
            InitializeComponent();
            vm = new ViewModel.EarthSatelliteViewModel();
            DataContext = vm;
#if SPHERE
            Terra = new Ellipse3D(1, 1, 1, 1000);
#else
            Terra = new EllipseRegion3D(1, 1, 1, 20, 50);
#endif
            PlotSphere(null, null);
        }

        public void OnViewportMouseDown(object sender, System.Windows.Input.MouseButtonEventArgs args)
        {
            Point pt = args.GetPosition(this.mainViewport);
            if (args.ChangedButton == MouseButton.Left)   // rotate or drag 3d model
            {
                m_transformMatrix.OnLBtnDown(pt);
            }
        }

        public void OnViewportMouseMove(object sender, System.Windows.Input.MouseEventArgs args)
        {
            Point pt = args.GetPosition(this.mainViewport);

            if (args.LeftButton == MouseButtonState.Pressed)   // rotate or drag 3d model
            {
                m_transformMatrix.OnMouseMove(pt, mainViewport);

                TransformChart();
            }
        }

        public void OnViewportMouseUp(object sender, System.Windows.Input.MouseButtonEventArgs args)
        {
            Point pt = args.GetPosition(this.mainViewport);
            if (args.ChangedButton == MouseButton.Left)
            {
                m_transformMatrix.OnLBtnUp();
            }
        }
        public void OnKeyDown(object sender, System.Windows.Input.KeyEventArgs args)
        {
            m_transformMatrix.OnKeyDown(args);
            TransformChart();
        }

        public void PlotSphere(object sender, RoutedEventArgs e)
        {
            //Random rnd = new Random();
            int nData = Terra.GetVertexNo();

            //for (int i = 0; i < nData; ++i)
            //{
            //    var p = Terra.GetPoint(i);
            //    Terra.SetColor(i, Color.FromScRgb(1.0f, 0, 0.2f + (float)Math.Acos(p.Z) / 5f, 0.2f + (float)Math.Acos(p.Z) / 5f));
            //}

            // Parallel.For(0, nData, i =>

            List<Vector3D> apexes = new List<Vector3D>
            {
                new Vector3D(0.110114001356683,0.020776017953791,0.0161249725195272),
                new Vector3D(0, 0, 0),
                new Vector3D(-0.110128429340776,-0.020679106196494,-0.0161472900825659),
                new Vector3D(0, 0, 0)
            };
            List<Vector3D> verts = new List<Vector3D>
            { 
                new Vector3D(0.286086225170584,-0.167067743709656,-0.94352691576839),
                new Vector3D(0.287704707730365,-0.177536763133048,-0.94111991737824),
                new Vector3D(0.0674393368684983,-0.21883933597819,-0.973427594056878),
                new Vector3D(0.0658668401225687,-0.208674988809425,-0.975764473844818)
            };
            SphericalGeom.Polygon sector = new SphericalGeom.Polygon(verts, apexes);
            Vector3D sun = new Vector3D(-99335404.3188502, 94380801.7803882, -53605586.5457589);
            var hemi = SphericalGeom.Polygon.Hemisphere(sun);
            var LitAndNot = SphericalGeom.Polygon.IntersectAndSubtract(sector, hemi);
            var sunP = vm.getPointPolygon(sun, 10);

            SphericalGeom.Polygon pol = new SphericalGeom.Polygon(
                new List<Vector3D>
                {
                    new Vector3D(-0.415256805379794,-0.0245434562815893,0.909373083139985),
                    new Vector3D(-0.423538220582993,-0.0185226831269007,0.905688846080801),
                    new Vector3D(-0.304617657598805,0.16455144919006,0.938152921036047),
                    new Vector3D(-0.296533516087929,0.158649692318033,0.941752700534447),
                    new Vector3D(-0.415252956838029,-0.0245432288162827,0.909374846670253)
                },
                new List<Vector3D>
                {
                    new Vector3D(-0.0593839609633907,-0.0915926626399865,-0.0161974938860543),
                    new Vector3D(0, 0, 0),
                    new Vector3D(0.0595345204650451,0.0914706217749444,0.0162654080488155),
                    new Vector3D(0, 0,0),
                    new Vector3D(0,0,0)
                }
            );

            for (int i = 0; i < nData; ++i)
            {
                var p = Terra.GetPoint(i);


                //bool flag = false;
                //int pol_ind = 0;
                //for (int pi = 0; pi < vm.polygons.Count; pi++)
                //{
                //    var pol = vm.polygons[pi];
                //    if (pol.Contains(new Vector3D(p.X, p.Y, p.Z)))
                //    {
                //        flag = true;
                //        pol_ind = pi;
                //        break;
                //    }
                //}

                //float polColor = (float)(pol_ind + 1) / vm.polygons.Count;

                //if (flag)
                //{
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, polColor, (float)(1 - polColor), 0.0f));
                //}
                //else if (vm.PointInPolygon(p.X, p.Y, p.Z))
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, 1.0f, 0.0f, 0.0f));
                //else if (vm.PointInCaptureInterval(p.X, p.Y, p.Z))
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.0f, 0.0f, 0.0f));
                //else if (vm.PointInLane(p.X, p.Y, p.Z))
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.0f, 0.0f, 0.0f));
                if (sector.Contains(new Vector3D(p.X, p.Y, p.Z)))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.0f, 0.0f, 1.0f));
                }
                //else if (LitAndNot.Item1[0].Contains(new Vector3D(p.X, p.Y, p.Z)))
                //{
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, 1.0f, 1.0f, 1.0f));
                //}
                //else if (LitAndNot.Item1[1].Contains(new Vector3D(p.X, p.Y, p.Z)))
                //{
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.75f, 0.75f, 0.75f));
                //}
                //else if (LitAndNot.Item2[0].Contains(new Vector3D(p.X, p.Y, p.Z)))
                //{
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.5f, 0.5f, 0.5f));
                //}
                //else if (LitAndNot.Item2[1].Contains(new Vector3D(p.X, p.Y, p.Z)))
                //{
                //    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.25f, 0.25f, 1.0f));
                //}
                else if (hemi[0].Contains(new Vector3D(p.X, p.Y, p.Z)))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 1.0f, 0.0f, 0.0f));
                }
                else if (hemi[1].Contains(new Vector3D(p.X, p.Y, p.Z)))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.0f, 1.0f, 0.0f));
                }
                else
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0, 0.2f + (float)Math.Acos(p.Z) / 5f, 0.2f + (float)Math.Acos(p.Z) / 5f));

            }
            // );

            ArrayList meshs = new ArrayList { Terra };

            Model3D model3d = new Model3D();
            m_nChartModelIndex = model3d.UpdateModel(meshs, null, m_nChartModelIndex, this.mainViewport);

            float viewRange = 2;
            m_transformMatrix.CalculateProjectionMatrix(-viewRange, viewRange, -viewRange, viewRange, -viewRange, viewRange, 0.5);
        }

        public void PlotSquares(object sender, RoutedEventArgs e)
        {

            var nData = Terra.GetVertexNo();
            //for (int i = 0; i < nData; ++i)
            //{
            //    var p = Terra.GetPoint(i);
            //    Terra.SetColor(i, Color.FromScRgb(1.0f, 0, 0.2f + (float)Math.Acos(p.Z) / 5f, 0.2f + (float)Math.Acos(p.Z) / 5f));
            //}
            for (int i = 0; i < nData; ++i)
            {
                var p = Terra.GetPoint(i);
                if (vm.Requests.Count > 0 && vm.PointInBoundingBox(p.X, p.Y, p.Z))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.0f, 1.0f, 0.0f));
                }
                else if (vm.Requests.Count > 0 && vm.PointInPolygon(p.X, p.Y, p.Z))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.5f, 0.5f, 0.0f));
                }
                else
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0, 0.2f + (float)Math.Acos(p.Z) / 5f, 0.2f + (float)Math.Acos(p.Z) / 5f));
            }

            ArrayList meshs = new ArrayList { Terra };

            Model3D model3d = new Model3D();
            m_nChartModelIndex = model3d.UpdateModel(meshs, null, m_nChartModelIndex, this.mainViewport);

            float viewRange = 2;
            m_transformMatrix.CalculateProjectionMatrix(-viewRange, viewRange, -viewRange, viewRange, -viewRange, viewRange, 0.5);
        }

        private void TransformChart()
        {
            if (m_nChartModelIndex == -1) return;
            ModelVisual3D visual3d = (ModelVisual3D)(this.mainViewport.Children[m_nChartModelIndex]);
            if (visual3d.Content == null) return;
            Transform3DGroup group1 = visual3d.Content.Transform as Transform3DGroup;
            group1.Children.Clear();
            group1.Children.Add(new MatrixTransform3D(m_transformMatrix.m_totalMatrix));
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            
        }

        private void Button_isRequestFeasible(object sender, RoutedEventArgs e)
        {
            
        }

        private void getCaptureConfArray_Click(object sender, RoutedEventArgs e)
        {
             
        }



    }

}
