/**
 * @author Carlos Tampier Cotoras - carlos.tampier.cotoras@fkie.fraunhofer.de
 *
 * Copyright (c) 2021 - Fraunhofer FKIE
 */

//DOMGeneric class for wrapping common interactions with the DOM.
export class DOMGeneric {
  static clearChildren(elementRef) {
    //Remove all html content from the referenced element.
    elementRef.innerHTML = "";
  }

  static addOption(selectRef, value, elementClass = null) {
    //Create and add an option to a select, with an optional html class.
    let opt = document.createElement("option");
    opt.value = value;
    opt.innerText = value;
    if (elementClass) {
      opt.classList.add(elementClass);
    }
    selectRef.appendChild(opt);
  }

  static removeFirstEmptyOption(selectRef) {
    //If the first option of a select is empty, remove it.
    if (selectRef.firstChild.value === "") {
      selectRef.removeChild(selectRef.firstChild);
    }
  }

  static populateSelect(selectRef, list, classList = null) {
    //Add options from a list to a select, with optional classes.
    //Also adds an empty option at the top.
    DOMGeneric.clearChildren(selectRef);
    DOMGeneric.addOption(selectRef, "");
    for (let i = 0; i < list.length; i++) {
      DOMGeneric.addOption(selectRef, list[i], classList && classList[i]);
    }
  }

  static addRow(tableRef, dataList) {
    //Add a row to a table body with the contents in the dataList array.
    const newRow = tableRef.insertRow(0);
    for (const value of dataList) {
      const newCell = newRow.insertCell();
      const newText = document.createTextNode(value);
      newCell.appendChild(newText);
    }
  }

  static populateTable(tableRef, dataMatrix) {
    //Clears table body and and adds all the data from the dataMatrix matrix.
    DOMGeneric.clearChildren(tableRef);
    for (const dataList of dataMatrix.reverse()) {
      DOMGeneric.addRow(tableRef, dataList);
    }
  }

  static addImage(elementRef, src, alt, ...extraProperties) {
    //Adds an image to the provided element.
    //The extraProperties argument allows for the optional setting of properties to the image element,
    //they must come in "propertyName", "propertyValue" pairs.
    let img = document.createElement("img");
    img.src = src;
    img.alt = alt;
    if (extraProperties) {
      if (extraProperties.length < 2 || extraProperties.length % 2 != 0) {
        elementRef.appendChild(img);
        return -1;
      }
      for (let i = 0; i < extraProperties.length; i = i + 2) {
        img[extraProperties[i]] = extraProperties[i + 1];
      }
    }
    elementRef.appendChild(img);
  }

  static createMaterialIcon(iconName) {
    //Creates an icon element and assigns some properties and styling.
    //The icons are actually inserted from the google material icons, that must be included in the html.
    const icon = document.createElement("i");
    icon.className = "material-icons";
    icon.textContent = iconName;
    icon.style.alignSelf = "center";
    if (iconName === "clear") {
      icon.style.color = "red";
    } else if (iconName === "done") {
      icon.style.color = "green";
    }
    return icon;
  }

  static createMaterialIconsContainer(containerTag, ...iconNames) {
    //Creates a container (of the specified type) with as many icons as specified in the arguments.
    //Assigns an icon-container html class to the container.
    //The icons are actually inserted from the google material icons, that must be included in the html.
    const container = document.createElement(containerTag);
    container.className = "icon-container";
    iconNames.forEach((iconName) => {
      container.appendChild(DOMGeneric.createMaterialIcon(iconName));
    });
    return container;
  }
}
