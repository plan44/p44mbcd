//
//  Copyright (c) 2019 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of p44utils.
//
//  p44utils is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  p44utils is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with p44utils. If not, see <http://www.gnu.org/licenses/>.
//

#ifndef __p44utils__uielements__
#define __p44utils__uielements__


#include "lvgl.hpp"
#include "jsonobject.hpp"

#if ENABLE_LVGL

using namespace std;

namespace p44 {

  class LvGLUi;

  /// base class for any configurable object
  class LvGLUIObject : public P44Obj
  {
  protected:

    LvGLUi& lvglui;
    string name;

  public:

    LvGLUIObject(LvGLUi& aLvGLUI) : lvglui(aLvGLUI) {};

    const string &getName() { return name; };

    /// configure this object from json
    /// @param aConfig JSON object containing configuration propertyname/values
    virtual ErrorPtr configure(JsonObjectPtr aConfig);

  };




  /// a initialized theme (base theme + hue + font)
  class LvGLUiTheme : public LvGLUIObject
  {
    typedef LvGLUIObject inherited;

  public:

    lv_theme_t* theme;

    LvGLUiTheme(LvGLUi& aLvGLUI) : inherited(aLvGLUI), theme(NULL) {};

    /// configure this object from json
    /// @param aConfig JSON object containing configuration propertyname/values
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;

  };
  typedef boost::intrusive_ptr<LvGLUiTheme> LvGLUiThemePtr;
  typedef std::map<string, LvGLUiThemePtr> ThemeMap;


  /// customized style
  class LvGLUiStyle : public LvGLUIObject
  {
    typedef LvGLUIObject inherited;

  public:

    lv_style_t style; ///< the LGVL style

    LvGLUiStyle(LvGLUi& aLvGLUI) : inherited(aLvGLUI) {};

    /// configure this object from json
    /// @param aConfig JSON object containing configuration propertyname/values
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;

  };
  typedef boost::intrusive_ptr<LvGLUiStyle> LvGLUiStylePtr;
  typedef std::map<string, LvGLUiStylePtr> StyleMap;
  typedef std::list<LvGLUiStylePtr> StyleList;


  // MARK: - element and container base classes

  class LVGLUiElement;
  class LVGLUiContainer;
  typedef boost::intrusive_ptr<LVGLUiElement> LVGLUiElementPtr;
  typedef boost::intrusive_ptr<LVGLUiContainer> LVGLUiContainerPtr;


  /// abstract base class for visible UI elements, wrapping a lv_obj
  class LVGLUiElement : public LvGLUIObject
  {
    typedef LvGLUIObject inherited;

  public:

    string text; ///< text (e.g. title)
    lv_obj_t* element;
    LVGLUiContainer* parentP;

    LVGLUiElement(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
    virtual ~LVGLUiElement();

    /// configure this object from json
    /// @param aConfig JSON object containing configuration propertyname/values
    /// @param aParent parent object, or NULL if none
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;

    /// clear this element (and all of its named and unnamed children)
    virtual void clear();

    /// @return true if the wrapper object must be kept around (e.g. because it needs to handle events)
    virtual bool wrapperNeeded() { return !getName().empty(); }; // simple objects need the wrapper only if they can be referenced

    /// @param aElementPath dot separated absolute path beginning at root container, or dot-prefixed relative path
    ///   (.elem = one of my subelements, ..elem=a sibling (element in my parent's container), ...=grandparent, etc.)
    /// @return requested element or NULL if none found
    /// @note leaf element can only use absolute or at least ..-prefixed paths
    virtual LVGLUiElementPtr namedElement(const string aElementPath);

  };
  typedef std::map<string, LVGLUiElementPtr> ElementMap;
  typedef std::list<LVGLUiElementPtr> ElementList;


  /// abstract class for lv_cont and similar objects with layout features
  class LvGLUiLayoutContainer : public LVGLUiElement
  {
    typedef LVGLUiElement inherited;
  public:
    LvGLUiLayoutContainer(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate) : inherited(aLvGLUI, aParentP, aTemplate) {};
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;
  };


  /// abstract for a UI element that can create contained child objects from config
  class LVGLUiContainer : public LvGLUiLayoutContainer
  {
    typedef LvGLUiLayoutContainer inherited;

    ElementMap namedElements; ///< the contained elements that have a name because the need to be referencable
    ElementList anonymousElements; ///< the contained elements that need to be around after configuration because they are actionable

  public:

    LVGLUiContainer(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate) : inherited(aLvGLUI, aParentP, aTemplate) {};

    /// configure this object from json
    /// @param aConfig JSON object containing configuration propertyname/values
    /// @param aParent parent object, or NULL if none
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;

    /// clear this element (and all of its named and unnamed children)
    virtual void clear() P44_OVERRIDE;

    /// @param aElementPath dot separated absolute path beginning at root container, or dot-prefixed relative path
    ///   (.elem = one of my subelements, ..elem=a sibling (element in my parent's container), ...=grandparent, etc.)
    /// @return requested element or NULL if none found
    virtual LVGLUiElementPtr namedElement(const string aElementPath) P44_OVERRIDE;

  protected:

    ErrorPtr addElements(JsonObjectPtr aElementConfigArray, LVGLUiContainer* aParent, bool aContainerByDefault);

  };


  // MARK: - specific UI elements

  class LvGLUiPlain : public LVGLUiElement
  {
    typedef LVGLUiElement inherited;
  public:
    LvGLUiPlain(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
  };


  class LvGLUiPanel : public LVGLUiContainer
  {
    typedef LVGLUiContainer inherited;
  public:
    LvGLUiPanel(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
  };



  class LvGLUiImage : public LVGLUiElement
  {
    typedef LVGLUiElement inherited;
  public:
    LvGLUiImage(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;
  };


  class LvGLUiLabel : public LVGLUiElement
  {
    typedef LVGLUiElement inherited;
  public:
    LvGLUiLabel(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;
  };


  class LvGLUiButton : public LvGLUiLayoutContainer
  {
    typedef LvGLUiLayoutContainer inherited;
  public:
    LvGLUiButton(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;
  };


  class LvGLUiImgButton : public LVGLUiElement
  {
    typedef LVGLUiElement inherited;
  public:
    LvGLUiImgButton(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;
  };


  class LvGLUiSlider : public LVGLUiElement
  {
    typedef LVGLUiElement inherited;
  public:
    LvGLUiSlider(LvGLUi& aLvGLUI, LVGLUiContainer* aParentP, lv_obj_t *aTemplate);
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;
  };




  // MARK: - LvGLUi

  class LvGLUi : public LVGLUiContainer
  {
    typedef LVGLUiContainer inherited;

    lv_disp_t *display; ///< the display this gui appears on

    StyleMap styles; ///< custom styles
    StyleList adhocStyles; ///< keeps ad hoc styles
    ThemeMap themes; ///< initialized themes (basic theme + hue + font)

  protected:

    virtual void clear() P44_OVERRIDE;

  public:

    LvGLUi();

    /// initialize for use with a specified display and create UI hierarchy from config
    ErrorPtr initForDisplay(lv_disp_t* aDisplay, JsonObjectPtr aInitialConfig);

    /// can be used to re-configure UI later (e.g. to add more screens)
    virtual ErrorPtr configure(JsonObjectPtr aConfig) P44_OVERRIDE;

    /// get named theme (from themes defined in config)
    lv_theme_t* namedTheme(const string aThemeName);

    /// get named style (custom as defined in config or built-in)
    lv_style_t* namedStyle(const string aStyleName);

    /// get named style from styles list or create ad-hoc style from definition
    lv_style_t* namedOrAdHocStyle(JsonObjectPtr aStyleNameOrDefinition, bool aDefaultToPlain);

    /// get image file path, will possibly look up in different places
    /// @param aImageSpec a path specifying an image
    virtual string imagePath(const string aImageSpec);

    /// get image source specification by name
    /// @note names containing dots will be considered file paths. Other texts are considered symbol names.
    ///    fallback is a text image label.
    string namedImageSource(const string& aImageSpec);

  };



} // namespace p44






#endif // ENABLE_LVGL

#endif /* __p44utils__uielements__ */
