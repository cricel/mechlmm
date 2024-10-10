import React, { useState, useEffect } from 'react';
import axios from 'axios';

interface ApiResponse {
  data: { [key: string]: any }[]; // A more generic structure to accommodate dynamic data
}

const App: React.FC = () => {
  const [tableData, setTableData] = useState<{ [key: string]: { [key: string]: any }[] }>({});
  const tableNames: string[] = ['objects_map', 'data_log']; // List of table names you need to render

  useEffect(() => {
    const fetchTableData = async () => {
      const fetchedData: { [key: string]: { [key: string]: any }[] } = {};
      for (const table of tableNames) {
        try {
          const response = await axios.get<ApiResponse>(`http://192.168.1.134:5001/database/get_table/${table}`);
          fetchedData[table] = response.data;
        } catch (error) {
          console.error(`Error fetching data for ${table}`, error);
        }
      }
      setTableData(fetchedData); // Update the state with the data for all tables
    };

    // Fetch data every 2 seconds
    const intervalId = setInterval(fetchTableData, 2000);

    // Cleanup the interval on component unmount
    return () => clearInterval(intervalId);
  }, [tableNames]);

  // Function to render nested arrays
  const renderNestedArray = (arr: any[]): JSX.Element => {
    return (
      <ul>
        {arr.map((item, index) => (
          <li key={index}>
            {Array.isArray(item) ? renderNestedArray(item) : item.toString()}
          </li>
        ))}
      </ul>
    );
  };

  // Dynamically render the table based on the data structure
  const renderTable = (data: { [key: string]: any }[]): JSX.Element | null => {
    if (!data || data.length === 0) return null; // Add check for empty or null data

    const headers = Object.keys(data[0]); // Extract headers dynamically from the first object

    return (
      <table border={1} cellPadding={5} style={{ borderCollapse: 'collapse', width: '100%' }}>
        <thead>
          <tr>
            {headers.map((header, index) => (
              <th key={index}>{header}</th>
            ))}
          </tr>
        </thead>
        <tbody>
          {data.map((row, rowIndex) => (
            <tr key={rowIndex}>
              {headers.map((header, cellIndex) => (
                <td key={cellIndex}>
                  {/* Render nested arrays or display the value */}
                  {Array.isArray(row[header]) ? renderNestedArray(row[header]) : row[header]?.toString() || 'N/A'}
                </td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    );
  };

  return (
    <div>
      <h1>DB Inspector</h1>
      {tableNames.map((table, index) => (
        <div key={index}>
          <h2>{table}</h2>
          {tableData[table] && tableData[table].length > 0 ? (
            <div>{renderTable(tableData[table])}</div>
          ) : (
            <p>Loading data for {table}...</p>
          )}
        </div>
      ))}
    </div>
  );
};

export default App;
