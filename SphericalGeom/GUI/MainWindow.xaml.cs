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
      //  public Ellipse3D Terra;
        public EllipseRegion3D Terra;
        private ViewModel.EarthSatelliteViewModel vm;

        public MainWindow()
        {
            InitializeComponent();
            vm = new ViewModel.EarthSatelliteViewModel();
            DataContext = vm;
           // Terra = new Ellipse3D(1, 1, 1, 200);            
            Terra = new EllipseRegion3D(1, 1, 1, 5, 150);
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
            for (int i = 0; i < nData; ++i)
            {
                var p = Terra.GetPoint(i);


                if (vm.PointInCaptureInterval(p.X, p.Y, p.Z))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.0f, 0.0f, 0.0f));
                }
                else if (vm.Requests.Count > 0 && vm.PointInRegion(p.X, p.Y, p.Z))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 0.5f, 0.5f, 0.0f));
                }
                else if (vm.PointInLane(p.X, p.Y, p.Z))
                {
                    Terra.SetColor(i, Color.FromScRgb(1.0f, 1.0f, 0.0f, 0.0f));
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
            vm.CreateCaptureIntervals();              
        }

        private void Button_isRequestFeasible(object sender, RoutedEventArgs e)
        {
            vm.test_isRequestFeasible();
        }

        private void getCaptureConfArray_Click(object sender, RoutedEventArgs e)
        {
            vm.test_getCaptureConfArray();
        }

 
         
    }

}
