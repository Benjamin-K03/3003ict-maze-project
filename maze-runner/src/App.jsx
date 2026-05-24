import { useState } from 'react'
import './App.css'
import MazeDesigner from "./MazeDesigner";

function App() {
    const [width, setWidth] = useState(10);
    const [height, setHeight] = useState(10);

    return (
        <MazeDesigner/>
    )
}

export default App

