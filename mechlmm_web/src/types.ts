// src/types.ts

export interface TableData {
    [key: string]: any; // Each row's column can be of any type (number, string, etc.)
  }
  
  export interface ApiResponse {
    data: TableData[];
  }
  