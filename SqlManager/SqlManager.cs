using System;
using System.Web; 
using System.Data.SqlClient;
using System.Data;
using System.Xml;
using System.Collections;
using DIOS.Common.Interfaces;
using System.Reflection;
using Microsoft.SqlServer.Types;

/// <summary> 
/// Summary description for SqlManager 
/// </summary> 
namespace DIOS.Common
{

    //public class CustomSqlManager : SqlManager
    //{
    //    public CustomSqlManager(string conn_string)
    //    {
    //        initConnectionString = conn_string;
    //        _sqlConnection = new SqlConnection();
    //        _sqlConnection.ConnectionString = GetConnectionString();
    //    }
    //}

    public class SqlManager
    {
        protected SqlTransaction _currentTransaction;
        public SqlTransaction CurrentTransaction
        {
            get
            {
                return _currentTransaction;
            }
        }
        private string userName = "";
        public virtual string UserName
        {
            get
            {
                return userName;
            }
        }

        protected string _sql_server_name = null;
        private string _sql_system_database_name = null;
        protected string _sql_database_name = null;
        protected string _sql_login = null;
        protected string _sql_password = null;

        private string initConnectionString;
        private SqlConnection _sqlConnection;
        public SqlConnection sqlConnection
        {
            get
            {
                return _sqlConnection;
            }
        }

        public void StartTransaction()
        {
            if (_currentTransaction != null)
                throw new Exception("Transaction already started");
            if (sqlConnection.State != System.Data.ConnectionState.Open)
                sqlConnection.Open();
            _currentTransaction = sqlConnection.BeginTransaction();
        }

        public void CommitTransaction()
        {
            if (_currentTransaction == null)
                throw new Exception("Transaction does not exist");
            if(sqlConnection.State != System.Data.ConnectionState.Open)
            {
                _currentTransaction = null;
                return;
            }
            _currentTransaction.Commit();
            _currentTransaction = null;
            sqlConnection.Close();
        }
        public void RollbackTransaction()
        {
            if (_currentTransaction == null)
                throw new Exception("Transaction does not exist");
            if (sqlConnection.State != System.Data.ConnectionState.Open)
            {
                _currentTransaction = null;
                return;
            }
            _currentTransaction.Rollback();
            _currentTransaction = null;
        }
        public string DatabaseName
        {
            get
            {
                return _sql_database_name;
            }
        }

        public string ServerName
        {
            get
            {
                return _sql_server_name;
            }
        }

        protected virtual bool Anonimous
        {
            get
            {
                return false;
            }
        }

        public SqlManager(string conn_string)
        {
            _sqlConnection = new SqlConnection();
            _sqlConnection.ConnectionString = conn_string;
        }
        public SqlManager()
        {
            if (HttpContext.Current != null)
            {
                userName = "";
                try
                {
                    userName = HttpContext.Current.User.Identity.Name;
                }
                catch { }
                if (userName == "" && !Anonimous)
                    throw new UnauthorizedAccessException("Операция запрещена для неавторизованных пользователей.");
            }
            string fileName = AppDomain.CurrentDomain.BaseDirectory + "web.config";
            if (!System.IO.File.Exists(fileName))
            {
                throw new Exception("Файл конфигурации " + fileName + " не существует!");
            }
            XmlDocument xmlDoc = new XmlDocument();
            xmlDoc.Load(fileName);
            XmlNode configNode = xmlDoc.DocumentElement["connectionStrings"];
            if (configNode == null) throw new NullReferenceException("В Файле конфигурации отсутствует узел <connectionStrings>");
            GetConnectionAttributes(configNode);

            _sqlConnection = new SqlConnection();
            _sqlConnection.ConnectionString = GetConnectionString();

        }

        protected bool _is_occupied = false;
        public bool IsOccupied
        {
            get
            {
                return _is_occupied;
            }
            set
            {
                _is_occupied = value;
            }
        }

        public void SetServerName(string serverName)
        {
            _sql_server_name = serverName;
        }
        public void SetDatabaseName(string dbName)
        {
            if (dbName == null)
                _sql_database_name = _sql_system_database_name;
            else
                _sql_database_name = dbName;
        }
        ~SqlManager()
        {
            if (sqlConnection != null)
            if (sqlConnection.State != System.Data.ConnectionState.Closed)
                sqlConnection.Close();
        }

        protected virtual XmlNode GetConnectionNode(XmlNode configNode)
        {
            return configNode.ChildNodes[0];
        }
        public void GetConnectionAttributes(XmlNode configNode)
        {
            XmlNode connNode = GetConnectionNode(configNode);
            if (connNode == null) throw new NullReferenceException("Узел <connectionStrings> не содержит потомков.");
            initConnectionString = connNode.Attributes["connectionString"].Value.ToString();

            string db_string = ";";
            if (initConnectionString.IndexOf("Database") >= 0)
                db_string = initConnectionString.Substring(initConnectionString.IndexOf("Database"));
            else if (initConnectionString.IndexOf("Initial Catalog") >= 0)
                db_string = initConnectionString.Substring(initConnectionString.IndexOf("Initial Catalog"));
            _sql_database_name = db_string.Split(';')[0].Split('=')[1];
            _sql_system_database_name = _sql_database_name;

            string srv_string = ";";
            if (initConnectionString.IndexOf("Server") >= 0)
                srv_string = initConnectionString.Substring(initConnectionString.IndexOf("Server"));
            else if (initConnectionString.IndexOf("Data Source") >= 0)
                srv_string = initConnectionString.Substring(initConnectionString.IndexOf("Data Source"));
            _sql_server_name = srv_string.Split(';')[0].Split('=')[1];

            string login_string = ";";
            string pwd_string = ";";
            try
            {
                if (initConnectionString.IndexOf("user") >= 0)
                    login_string = initConnectionString.Substring(initConnectionString.IndexOf("user"));
                _sql_login = login_string.Split(';')[0].Split('=')[1];

                if (initConnectionString.IndexOf("password") >= 0)
                    pwd_string = initConnectionString.Substring(initConnectionString.IndexOf("password"));
                _sql_password = pwd_string.Split(';')[0].Split('=')[1];
            }
            catch { }
        }

        public virtual string GetDbPassword(string login)
        {
            return login.Replace("_", "") + "123456" + login.ToUpper().Replace("_", "");
        }

        protected virtual string GetConnectionString()
        {
            if (_sql_login == null || _sql_login == "")
                //return string.Format("Data Source={0};Initial Catalog={1};Integrated Security=SSPI", _sql_server_name, _sql_database_name);
                return string.Format("Server={0};Database={1};user={2};password={3}", _sql_server_name, _sql_database_name, UserName, GetDbPassword(UserName));
            else
                return string.Format("Server={0};Database={1};user={2};password={3}", _sql_server_name, _sql_database_name.Replace("[", "").Replace("]", ""), _sql_login, _sql_password);
        }

        public System.Data.DataTable ExecSql(string query)
        {
            return ExecSql(query, null);
        }

        public int ExecuteSPWithResult(string sp_name, bool add_db_name, IParameterCollection Params)
        {
            //SqlConnection sqlConn = new SqlConnection();
            //sqlConn.ConnectionString = GetConnectionString();
            if (add_db_name)
                sp_name = this._sql_database_name + ".dbo." + sp_name;

            SqlCommand cmdCommand = new SqlCommand();
            cmdCommand.Connection = _sqlConnection;
            cmdCommand.CommandType = CommandType.StoredProcedure;
            SqlParameter RetVal = cmdCommand.Parameters.Add("RetVal", SqlDbType.Int);
            RetVal.Direction = ParameterDirection.ReturnValue;
            if (Params != null)
            {
                foreach (IParameter Param in Params)
                {
                    SqlDbType paramType = System.Data.SqlDbType.NVarChar;
                    object paramValue = Param.Value;
                    if (Param.Value.GetType() == typeof(SqlInt32))
                    {
                        paramType = SqlDbType.Int;
                        paramValue = ((SqlInt32)Param.Value).Value;
                    }
                    SqlParameter sqlParam = cmdCommand.Parameters.Add(Param.Name, paramType);
                    sqlParam.Value = paramValue;
                    if (Param.Predicate == "out")
                    {
                        sqlParam.Direction = ParameterDirection.Output;
                        sqlParam.SqlDbType = SqlDbType.Int;
                    }
                }
            }
            cmdCommand.CommandText = sp_name;
            try
            {
                this.sqlConnection.Open();
                cmdCommand.ExecuteNonQuery();
                return (int)RetVal.Value; 
            }
            catch (Exception ex)
            {
                throw ex;
            }
            finally
            {
                this.sqlConnection.Close();
            }
        }

        public System.Data.DataTable ExecSql(string query, IParameterCollection Params)
        {
            return ExecSql(query, Params, false);
        }
        public System.Data.DataTable ExecSql(string query, IParameterCollection Params, bool resetConnection)
        {
            if (resetConnection)
                _sqlConnection = null;
            System.Data.DataSet testDataset = null;
            if (_sqlConnection == null)
            {
                _sqlConnection = new SqlConnection();
                _sqlConnection.ConnectionString = GetConnectionString();
            }

            SqlCommand cmdCommand = new SqlCommand();
            cmdCommand.Connection = _sqlConnection;
            cmdCommand.Transaction = _currentTransaction;
            cmdCommand.CommandType = Params == null? System.Data.CommandType.Text : System.Data.CommandType.StoredProcedure;
            if(Params != null)
            {
                foreach (IParameter Param in Params)
                {
                    System.Data.SqlDbType paramType = System.Data.SqlDbType.NVarChar;
                    object paramValue = Param.Value;
                    if(Param.Value.GetType() == typeof(SqlInt32))
                    {
                        paramType = System.Data.SqlDbType.Int;
                        paramValue = ((SqlInt32)Param.Value).Value;
                    }
                    SqlParameter sqlParam = cmdCommand.Parameters.Add(Param.Name, paramType);
                    sqlParam.Value = paramValue;
                    if (Param.Predicate == "out")
                    {
                        sqlParam.Direction = System.Data.ParameterDirection.Output;
                        sqlParam.SqlDbType = System.Data.SqlDbType.Int;
                    }
                }
            }
            cmdCommand.CommandText = query;

            SqlDataAdapter daAdapter = new SqlDataAdapter();
            daAdapter.SelectCommand = cmdCommand;
            testDataset = new System.Data.DataSet();

            try
            {
                if(_currentTransaction == null)
                    _sqlConnection.Open();
                daAdapter.Fill(testDataset);
                if (Params != null)
                {
                    foreach (IParameter Param in Params)
                    {
                        if (Param.Predicate == "out")
                        {
                            SqlParameter sqlParam = cmdCommand.Parameters[Param.Name];
                            Param.Value = sqlParam.Value;
                        }
                    }
                }
                if (testDataset.Tables.Count > 0)
                {
                    return testDataset.Tables[0];
                }
                else
                    return null;
            }
            catch (Exception ex)
            {
                throw ex;
            }
            finally
            {
                if (_currentTransaction == null)
                    _sqlConnection.Close();
            }
        }

        public System.Data.DataTable GetSqlObject(string ClassName, string Where)
        {
            return ExecSql(string.Format("select top 1000 * from {0}(nolock) {1}", ClassName, Where));
        }

        public System.Data.DataTable GetSqlObject(string ClassName, string KeyName, string ID)
        {
            return GetSqlObject(ClassName, string.Format("where {0} = {1}", KeyName, ID));
        }

        public System.Data.DataTable ExecuteSPWithParams(string sp_name, IParameterCollection Params)
        {
            return ExecSql(sp_name, Params);
        }
        public System.Data.DataTable ExecuteSPWithParams(string sp_name, object[] Params)
        {
            return ExecuteSPWithParams(sp_name, false, Params);
        }
        public System.Data.DataTable ExecuteSPWithParams(string sp_name, bool add_db_name, object[] Params)
        {
            if (add_db_name)
                sp_name = this._sql_database_name + ".dbo." + sp_name;
            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            sb.Append("exec ");
            sb.Append(sp_name);
            for (int p = 0; p < Params.Length; p++)
            {
                if (p > 0)
                    sb.Append(",");
                sb.Append(" ");
                bool needApostrophe = Params[p] != null && Params[p].GetType() == typeof(string);
                string ParamValue = "null";
                if (Params[p] != null)
                    ParamValue = Params[p].ToString();
                ParamValue = ParamValue.Replace("'", "''");
                if (needApostrophe)
                    sb.Append("'");
                sb.Append(ParamValue);
                if (needApostrophe)
                    sb.Append("'");
            }
            return ExecSql(sb.ToString());
        }


        public IndexerPropertyDescriptorCollection GetObjectProperties(Type ObjectType)
        {
            IndexerPropertyDescriptorCollection _objectProperties = IndexerPropertyDescriptorCollection.Empty;
            foreach (PropertyInfo pInfo in ObjectType.GetProperties())
            {
                ObjectPropertyAttribute objectPropertyAttribute = (System.Attribute.GetCustomAttribute(pInfo, typeof(ObjectPropertyAttribute)) as ObjectPropertyAttribute);
                if (objectPropertyAttribute != null)
                {
                    _objectProperties.Add(new IndexerPropertyDescriptor(pInfo.Name, System.Attribute.GetCustomAttributes(pInfo), objectPropertyAttribute.DisplayName, pInfo.PropertyType));
                }
            }
            return _objectProperties;
        }

    }

}