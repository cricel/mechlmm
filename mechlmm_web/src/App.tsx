import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { TableData, ApiResponse } from './types';

const App: React.FC = () => {
  const [tableData, setTableData] = useState<{ [key: string]: TableData }>({});
  const tableNames: string[] = ['objects_map']; // Table name

  useEffect(() => {
    const fetchTableData = async () => {
      const data: { [key: string]: TableData } = {};
      for (let table of tableNames) {
        try {
          // Fetch data from updated API URL
          const response = await axios.get<ApiResponse>(`http://192.168.1.134:5001/database/get_table/${table}`);
          data[table] = response.data;
        } catch (error) {
          console.error(`Error fetching data for ${table}`, error);
        }
      }
      setTableData(data);
    };

    fetchTableData();
  }, []);

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

  // Render the data in a table format
  const renderTable = (obj: { [key: string]: any }): JSX.Element => {
    const headers = Object.keys(obj); // Extract headers from object keys
    const rows = Object.values(obj);  // Extract data from object values

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
          <tr>
            {rows.map((row, rowIndex) => (
              <td key={rowIndex}>
                {/* Render nested arrays properly */}
                {Array.isArray(row) ? renderNestedArray(row) : row.toString()}
              </td>
            ))}
          </tr>
        </tbody>
      </table>
    );
  };

  return (
    <div>
      <h1>PostgreSQL Table Data</h1>
      {tableNames.map((table) => (
        <div key={table}>
          <h2>{table}</h2>
          {tableData[table] ? (
            <div>
              {/* Render the object content in table format */}
              {renderTable(tableData[table])}
            </div>
          ) : (
            <p>Loading data...</p>
          )}
        </div>
      ))}
    </div>
  );
};

export default App;
